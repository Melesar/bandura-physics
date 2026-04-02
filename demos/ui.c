#define CLAY_IMPLEMENTATION
#define RAYGUI_IMPLEMENTATION

#include "core.h"
#include "raygui.h"
#include "cyber/style_cyber.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <float.h>

struct {
  uint8_t *memory;
  uint32_t pointer;
} custom_arena;

uint8_t *clay_memory;

void *arena_alloc(uint32_t size) {
  uint8_t *buffer = custom_arena.memory + custom_arena.pointer;
  custom_arena.pointer += size;
  return buffer;
}

static void clay_error_handler(Clay_ErrorData error_data) {
  if (error_data.errorType == CLAY_ERROR_TYPE_DUPLICATE_ID) {
    return;
  }

  char buffer[250];

  uint32_t text_len = error_data.errorText.length;
  strncpy(buffer, error_data.errorText.chars, text_len);
  buffer[text_len < 250 ? error_data.errorText.length : 249] = 0;

  TraceLog(LOG_ERROR, "Clay error: %s", buffer);
}

static uint32_t memory_size_aligned(uint32_t required) {
  uint32_t unaligned = required % 8;
  if (unaligned) {
    required += 8 - unaligned;
  }

  return required;
}

static Rectangle clay_rect(Clay_BoundingBox bb) {
  return (Rectangle) { bb.x, bb.y, bb.width, bb.height };
}

static Clay_Dimensions clay_screen_dimensions() {
  return (Clay_Dimensions){ .height = GetScreenHeight(), .width = GetScreenWidth() };
}

static Clay_String clay_from_string(const char *label) {
  return (Clay_String) { .chars = label, .length = strlen(label), .isStaticallyAllocated = true };
}

static Clay_String clay_string_concat(const char *a, const char *b) {
  uint32_t len_a = strlen(a);
  uint32_t len_b = strlen(b);

  uint32_t required_length = memory_size_aligned(len_a + len_b + 2);
  char *s = arena_alloc(required_length);

  strncpy(s, a, len_a);
  s[len_a] = '_';
  strncpy(s + len_a + 1, b, len_b);
  s[len_a + len_b + 1] = 0;

  return (Clay_String) { .chars = s, .length = len_a + len_b + 1, .isStaticallyAllocated = false };
}

static Clay_Color clay_element_color(int control, int property_base, GuiState state) {
  Color color = GetColor(GuiGetStyle(control, property_base + 3 * state));

  return (Clay_Color) { color.r, color.g, color.b, color.a };
}

static Color clay_color_to_ray(Clay_Color color) {
  return (Color) { color.r, color.g, color.b, color.a };
}

static GuiState clay_gui_state() {
  Clay_PointerDataInteractionState pointer_state = Clay_GetPointerState().state;
  bool is_hovering = Clay_Hovered();

  if (is_hovering && pointer_state == CLAY_POINTER_DATA_PRESSED) {
    return STATE_PRESSED;
  } else if (is_hovering) {
    return STATE_FOCUSED;
  } else {
    return STATE_NORMAL;
  }
}

static bool clay_is_clicked() {
  Clay_PointerDataInteractionState pointer_state = Clay_GetPointerState().state;
  bool is_hovering = Clay_Hovered();

  return is_hovering && pointer_state == CLAY_POINTER_DATA_RELEASED_THIS_FRAME;
}

static char *clay_to_string(Clay_StringSlice slice) {
  uint32_t required_size = memory_size_aligned(slice.length + 1);

  char *string = arena_alloc(required_size);
  strncpy(string, slice.chars, slice.length);
  string[slice.length] = 0;

  return string;
}

static Clay_Dimensions measure_text(Clay_StringSlice slice, Clay_TextElementConfig *config, void *user_data) {
  char *string = clay_to_string(slice);

  return (Clay_Dimensions) {
    .width = GuiGetTextWidth(string),
    .height = GuiGetStyle(DEFAULT, TEXT_SIZE),
  };
}

static Clay_Sizing clay_container_sizing() {
  return (Clay_Sizing) { .width = CLAY_SIZING_GROW(100, 300) };
}

void ui_initialize() {
  uint32_t memory_size = Clay_MinMemorySize();
  clay_memory = malloc(memory_size);
  Clay_Arena arena = Clay_CreateArenaWithCapacityAndMemory(memory_size, clay_memory);
  Clay_Initialize(arena,
    clay_screen_dimensions(),
    (Clay_ErrorHandler) { .errorHandlerFunction = clay_error_handler, .userData = NULL });
  Clay_SetMeasureTextFunction(measure_text, NULL);
  Clay_SetDebugModeEnabled(true);

  GuiLoadStyleCyber();
  GuiSetStyle(DEFAULT, TEXT_PADDING, 0);


  custom_arena.memory = malloc(1 << 20);
}

void ui_teardown() {
  free(clay_memory);
  free(custom_arena.memory);
}

void ui_begin() {
  Clay_SetLayoutDimensions(clay_screen_dimensions());

  Vector2 mouse_pos = GetMousePosition();
  Vector2 scroll_wheel = GetMouseWheelMoveV();
  Clay_SetPointerState((Clay_Vector2) { mouse_pos.x, mouse_pos.y }, IsMouseButtonDown(MOUSE_LEFT_BUTTON));
  Clay_UpdateScrollContainers(true, (Clay_Vector2){ scroll_wheel.x, scroll_wheel.y }, 0);

  Clay_BeginLayout();
  Clay__OpenElementWithId(CLAY_ID("Container"));
  Clay__ConfigureOpenElement((Clay_ElementDeclaration) {
    .layout = {
      .layoutDirection = CLAY_TOP_TO_BOTTOM,
      .padding = CLAY_PADDING_ALL(15),
      .childGap = 15,
    }
  });
}

void ui_end(float dt) {
  Clay__CloseElement();

  Clay_RenderCommandArray commands = Clay_EndLayout(dt);
  for (int32_t i = 0; i < commands.length; ++i) {
    Clay_RenderCommand *command = &commands.internalArray[i];
    Rectangle rect = clay_rect(command->boundingBox);

    char *string;
    switch(command->commandType) {
      case CLAY_RENDER_COMMAND_TYPE_TEXT:
        string = clay_to_string(command->renderData.text.stringContents);
        GuiDrawText(string, rect, TEXT_ALIGN_LEFT, clay_color_to_ray(command->renderData.text.textColor));
        break;

      case CLAY_RENDER_COMMAND_TYPE_RECTANGLE:
        GuiDrawRectangle(rect, 0, BLANK, clay_color_to_ray(command->renderData.border.color));
        break;

      case CLAY_RENDER_COMMAND_TYPE_BORDER:
        GuiDrawRectangle(rect, command->renderData.border.width.left, clay_color_to_ray(command->renderData.border.color), BLANK);
        break;

      case CLAY_RENDER_COMMAND_TYPE_SCISSOR_START:
        BeginScissorMode((int)roundf(rect.x), (int)roundf(rect.y), (int)roundf(rect.width), (int)roundf(rect.height));
        break;

      case CLAY_RENDER_COMMAND_TYPE_SCISSOR_END:
        EndScissorMode();
        break;

      default:
        TraceLog(LOG_WARNING, "Unknown Clay render command: %d", command->commandType);
        break;
    }
  }

  custom_arena.pointer = 0;
}

bool ui_begin_area(const char *title, bool *collapsed) {
  Clay_String s = clay_from_string(title);

  Clay__OpenElementWithId(CLAY_SID(s));
  Clay__ConfigureOpenElement((Clay_ElementDeclaration) {
    .layout = {
      .sizing = { .width = CLAY_SIZING_FIT(300, FLT_MAX), .height = CLAY_SIZING_FIT(0, FLT_MAX) },
      .layoutDirection = CLAY_TOP_TO_BOTTOM,
    },
    .backgroundColor = clay_element_color(DEFAULT, BACKGROUND_COLOR, STATE_NORMAL),
    .border = { .width = { 1, 1, 1, 1, 0 }, .color = clay_element_color(DEFAULT, LINE_COLOR, STATE_NORMAL) }
  });

  CLAY(CLAY_SID(clay_string_concat(title, "statusbar")), {
    .layout = {
      .sizing = { .height = CLAY_SIZING_FIXED(20), .width = CLAY_SIZING_GROW() },
      .padding = { .left = 10, .right = 2 },
      .childAlignment = { .y = CLAY_ALIGN_Y_CENTER }
    },
    .backgroundColor = clay_element_color(STATUSBAR, BASE, STATE_NORMAL),
  }) {
    CLAY(CLAY_SID(clay_string_concat(title, "text")), {
      .layout = { .sizing = { .width = CLAY_SIZING_GROW(0) } }
    }) {
      CLAY_TEXT(s, { .textColor = clay_element_color(STATUSBAR, TEXT, STATE_NORMAL) });
    }

    CLAY(CLAY_SID(clay_string_concat(title, "close_btn")), {
      .layout = {
        .sizing = { .width = CLAY_SIZING_FIXED(18), .height = CLAY_SIZING_FIXED(18) },
        .childAlignment = { .y = CLAY_ALIGN_Y_CENTER },
      },
      .backgroundColor = clay_element_color(BUTTON, BASE, clay_gui_state()),
    }){
      if (clay_is_clicked()) {
        *collapsed = !*collapsed;
      }

      CLAY_TEXT(clay_from_string(GuiIconText(ICON_BOX_MINUS_FILL, NULL)), { .textColor = clay_element_color(BUTTON, TEXT, clay_gui_state()), .textAlignment = CLAY_TEXT_ALIGN_CENTER });
    };
  }

  float vertical_padding = *collapsed ? 0 : 15;
  Clay__OpenElementWithId(CLAY_SID(clay_string_concat(title, "content")));
  Clay__ConfigureOpenElement((Clay_ElementDeclaration) {
    .layout = {
      .sizing = { .width = CLAY_SIZING_GROW() },
      .layoutDirection = CLAY_TOP_TO_BOTTOM,
      .childGap = 10,
      .padding = { .left = 10, .right = 10, .top = vertical_padding, .bottom = vertical_padding },
    },
  });

  return !*collapsed;
}

void ui_end_area() {
  Clay__CloseElement(); // Children container
  Clay__CloseElement(); // Modal window
}

void ui_label(char *label) {
  CLAY_TEXT(clay_from_string(label), { .textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL) });
}

static void ui_value_label(const char *label, Clay_String value) {
  CLAY(CLAY_SID(clay_string_concat(label, "container")), {
    .layout = {
      .sizing = clay_container_sizing(),
      .layoutDirection = CLAY_LEFT_TO_RIGHT,
    },
  }) {
    CLAY(CLAY_SID(clay_string_concat(label, "label")), {
      .layout = { .sizing = { .width = CLAY_SIZING_GROW() } }
    }){
      CLAY_TEXT(clay_from_string(label), { .textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL) });
    };

    CLAY(CLAY_SID(clay_string_concat(label, "value_container")), {
      .layout = {
        .childAlignment = { .x = CLAY_ALIGN_X_RIGHT },
        .sizing = { .width = CLAY_SIZING_GROW() },
      }
    }) {

      CLAY_TEXT(value, { .textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL) });
    }
  }
}

void ui_label_v3(const char *label, v3 value) {
    char *v = arena_alloc(80); // Allocate more to keep the arena 8-bytes aligned
    snprintf(v, 80, "(%.2f, %.2f, %.2f)", value.x, value.y, value.z);
    Clay_String vs = { .chars = v, .length = strlen(v), .isStaticallyAllocated = false };

    ui_value_label(label, vs);
}


void ui_label_float(char *label, float value) {
  char *v = arena_alloc(80);
  snprintf(v, 80, "%.2f", value);
  Clay_String vs = { .chars = v, .length = strlen(v), .isStaticallyAllocated = false };

  ui_value_label(label, vs);
}

void ui_checkbox(const char *label, bool *is_checked) {
  CLAY(CLAY_SID(clay_from_string(label)), {
    .layout = {
      .sizing = clay_container_sizing(),
      .layoutDirection = CLAY_LEFT_TO_RIGHT,
      .childGap = 10,
      .childAlignment = { .y = CLAY_ALIGN_Y_CENTER }
    },
  }) {
    if (clay_is_clicked()) {
      *is_checked = !*is_checked;
    }

    GuiState state = clay_gui_state();
    float text_size = 1.2 * GuiGetStyle(DEFAULT, TEXT_SIZE);

    CLAY(CLAY_SID(clay_string_concat(label, "check")), {
      .layout = {
        .sizing = { .width = CLAY_SIZING_FIXED(text_size), .height = CLAY_SIZING_FIXED(text_size) },
      },
      .border = { .width = { 2, 2, 2, 2, 0 }, .color = clay_element_color(CHECKBOX, BORDER, state) },
      .backgroundColor = clay_element_color(CHECKBOX, TEXT, state),
    }) {
      if (*is_checked)
        CLAY_TEXT(clay_from_string(GuiIconText(ICON_BOX_CIRCLE_MASK, NULL)), { .textColor = clay_element_color(LABEL, TEXT, STATE_DISABLED), .textAlignment = CLAY_TEXT_ALIGN_CENTER });
    };

    CLAY_TEXT(clay_from_string(label), { .textColor = clay_element_color(CHECKBOX, TEXT, state) });
  }
}

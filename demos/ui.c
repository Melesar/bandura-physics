#define CLAY_IMPLEMENTATION
#define RAYGUI_IMPLEMENTATION

#include "core.h"
#include "raygui.h"
#include "cyber/style_cyber.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

typedef enum {
  UI_MODAL,
  UI_CHECKBOX,
} ui_element_type;

typedef struct {
  ui_element_type type;
  union {
    struct { const char *title; } modal;
    struct { bool *enabled; GuiState state; } checkbox;
  };
} custom_ui_element;

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
  char buffer[250];

  uint32_t text_len = error_data.errorText.length;
  strncpy(buffer, error_data.errorText.chars, text_len);
  buffer[text_len < 250 ? error_data.errorText.length : 249] = 0;

  TraceLog(LOG_ERROR, "Clay error: %s", buffer);
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

static char *clay_to_string(Clay_StringSlice slice) {
  uint32_t required_size = slice.length + 1;
  uint32_t unaligned = required_size % 8;
  if (unaligned) {
    required_size += 8 - unaligned;
  }

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

  GuiLoadStyleCyber();

  GuiSetStyle(DEFAULT, BORDER_WIDTH, 0);
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
  Clay_SetPointerState((Clay_Vector2) { mouse_pos.x, mouse_pos.y }, IsMouseButtonDown(MOUSE_LEFT_BUTTON));

  Clay_BeginLayout();
}

void ui_end(float dt) {
  Clay_RenderCommandArray commands = Clay_EndLayout(dt);
  for (int32_t i = 0; i < commands.length; ++i) {
    Clay_RenderCommand *command = &commands.internalArray[i];

    char *string;
    GuiState state = STATE_NORMAL;
    custom_ui_element custom_element;
    if (command->userData != NULL) {
      custom_element = *(custom_ui_element*) command->userData;
      if (custom_element.type == UI_CHECKBOX) {
        state = custom_element.checkbox.state;
      }
    }
    switch(command->commandType) {
      case CLAY_RENDER_COMMAND_TYPE_CUSTOM:
        custom_element = *(custom_ui_element*) command->renderData.custom.customData;
        switch(custom_element.type) {
          case UI_MODAL:
            GuiWindowBox(clay_rect(command->boundingBox), custom_element.modal.title);
            break;

          case UI_CHECKBOX:
            GuiDrawRectangle(clay_rect(command->boundingBox), 0, BLANK, GetColor(GuiGetStyle(CHECKBOX, TEXT + (3 * state))));
            if (*custom_element.checkbox.enabled) {
              GuiDrawRectangle(clay_rect(command->boundingBox), 0, BLANK, GetColor(GuiGetStyle(CHECKBOX, TEXT_COLOR_PRESSED)));
            }
            break;
        }
        break;

      case CLAY_RENDER_COMMAND_TYPE_TEXT:
        string = clay_to_string(command->renderData.text.stringContents);

        GuiDrawText(string, clay_rect(command->boundingBox), TEXT_ALIGN_LEFT, GetColor(GuiGetStyle(LABEL, TEXT + (3*state))));
        break;

      case CLAY_RENDER_COMMAND_TYPE_RECTANGLE:
        break;

      case CLAY_RENDER_COMMAND_TYPE_BORDER:
        GuiDrawRectangle(clay_rect(command->boundingBox), command->renderData.border.width.left, GetColor(GuiGetStyle(DEFAULT, BORDER + (3*state))), BLANK);
        break;

      default:
        TraceLog(LOG_INFO, "Clay render command: %d", command->commandType);
        break;
    }
  }

  custom_arena.pointer = 0;
}

void ui_begin_modal(const char *title) {
  custom_ui_element *custom_data = arena_alloc(sizeof(custom_ui_element));
  custom_data->type = UI_MODAL;
  custom_data->modal.title = title;

  Clay_String s = { .isStaticallyAllocated = true, .chars = title, .length = strlen(title) };
  Clay__OpenElementWithId(CLAY_SID(s));
  Clay__ConfigureOpenElement((Clay_ElementDeclaration) {
    .layout = {
      .sizing = { .width = CLAY_SIZING_FIT(500), .height = CLAY_SIZING_FIT(100) },
      .layoutDirection = CLAY_TOP_TO_BOTTOM,
    },
    .floating = { .attachTo = CLAY_ATTACH_TO_ROOT, .attachPoints = { .element = CLAY_ATTACH_POINT_LEFT_TOP, .parent = CLAY_ATTACH_POINT_LEFT_TOP } },
    .custom = { .customData = custom_data },
  });

  CLAY_AUTO_ID({
    .layout = { .sizing = { .height = CLAY_SIZING_FIXED(20), .width = CLAY_SIZING_GROW() } },
  }) {}

  Clay__OpenElement();
  Clay__ConfigureOpenElement((Clay_ElementDeclaration) {
    .layout = {
      .sizing = { .width = CLAY_SIZING_GROW() },
      .layoutDirection = CLAY_TOP_TO_BOTTOM,
      .childGap = 20,
      .padding = { .left = 10, .right = 10, .top = 15, },
    },
    .backgroundColor = { 255, 0, 0, 255 },
  });
}

void ui_end_modal() {
  Clay__CloseElement(); // Children container
  Clay__CloseElement(); // Modal window
}

void ui_label_v3(const char *label, v3 value) {
  CLAY_AUTO_ID({
    .layout = {
      .sizing = clay_container_sizing(),
      .layoutDirection = CLAY_LEFT_TO_RIGHT,
    },
  }) {
    CLAY_AUTO_ID({
      .layout = { .sizing = { .width = CLAY_SIZING_GROW() } }
    }){
      CLAY_TEXT(clay_from_string(label));
    };

    CLAY_AUTO_ID({
      .layout = {
        .childAlignment = { .x = CLAY_ALIGN_X_RIGHT },
        .sizing = { .width = CLAY_SIZING_GROW() },
      }
    }) {
      char *v = arena_alloc(24); // Allocate more to keep the arena 8-bytes aligned
      snprintf(v, 19, "(%.2f, %.2f, %.2f)", value.x, value.y, value.z);
      Clay_String vs = { .chars = v, .length = strlen(v), .isStaticallyAllocated = false };

      CLAY_TEXT(vs);
    }
  }
}

void ui_checkbox(const char *label, bool *is_checked) {
  custom_ui_element *checkbox = arena_alloc(sizeof(custom_ui_element));
  checkbox->type = UI_CHECKBOX;
  checkbox->checkbox.enabled = is_checked;

  CLAY(CLAY_SID(clay_from_string(label)), {
    .layout = {
      .sizing = clay_container_sizing(),
      .layoutDirection = CLAY_LEFT_TO_RIGHT,
      .childGap = 20,
      .padding = CLAY_PADDING_ALL(15),
    },
  }) {
    Clay_PointerDataInteractionState pointer_state = Clay_GetPointerState().state;
    bool is_hovering = Clay_Hovered();

    if (is_hovering && pointer_state == CLAY_POINTER_DATA_RELEASED_THIS_FRAME) {
      *is_checked = !*is_checked;
    }

    if (is_hovering && pointer_state == CLAY_POINTER_DATA_PRESSED) {
      checkbox->checkbox.state = STATE_PRESSED;
    } else if (is_hovering) {
      checkbox->checkbox.state = STATE_FOCUSED;
    } else {
      checkbox->checkbox.state = STATE_NORMAL;
    }

    float text_size = 1.2 * GuiGetStyle(DEFAULT, TEXT_SIZE);

    CLAY(CLAY_ID("check"), {
      .layout = {
        .sizing = { .width = CLAY_SIZING_FIXED(text_size), .height = CLAY_SIZING_FIXED(text_size) },
      },
      .border = { .width = { 2, 2, 2, 2, 0 } },
      .custom = { .customData = checkbox }
    });

    CLAY_TEXT(clay_from_string(label), { .userData = checkbox });
  }
}

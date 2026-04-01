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
    struct { bool *enabled; bool hovered; } checkbox;
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

static Clay_String clay_string(const char *label) {
  return (Clay_String) { .chars = label, .length = strlen(label), .isStaticallyAllocated = true };
}

static Clay_Dimensions measure_text(Clay_StringSlice slice, Clay_TextElementConfig *config, void *user_data) {
  uint32_t required_size = slice.length + 1;
  uint32_t unaligned = required_size % 8;
  if (unaligned) {
    required_size += 8 - unaligned;
  }

  char *string = arena_alloc(required_size);
  strncpy(string, slice.chars, slice.length);
  string[slice.length] = 0;

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
  Clay_SetPointerState((Clay_Vector2) { mouse_pos.x, mouse_pos.y }, IsMouseButtonPressed(MOUSE_LEFT_BUTTON));

  Clay_BeginLayout();
}

void ui_end(float dt) {
  Clay_RenderCommandArray commands = Clay_EndLayout(dt);
  for (int32_t i = 0; i < commands.length; ++i) {
    Clay_RenderCommand *command = &commands.internalArray[i];

    char *string;
    custom_ui_element custom_element;
    switch(command->commandType) {
      case CLAY_RENDER_COMMAND_TYPE_CUSTOM:
        custom_element = *(custom_ui_element*) command->renderData.custom.customData;
        switch(custom_element.type) {
          case UI_MODAL:
            GuiWindowBox(clay_rect(command->boundingBox), custom_element.modal.title);
            break;

          case UI_CHECKBOX:
            TraceLog(LOG_INFO, "Checkbox %d, enabled: %d, hovered: %d", command->id, *custom_element.checkbox.enabled, custom_element.checkbox.hovered);
            break;
        }
        break;

      case CLAY_RENDER_COMMAND_TYPE_TEXT:
        string = arena_alloc(command->renderData.text.stringContents.length + 1);
        strncpy(string, command->renderData.text.stringContents.chars, command->renderData.text.stringContents.length);
        string[command->renderData.text.stringContents.length] = 0;

        GuiLabel(clay_rect(command->boundingBox), string);
        break;

      case CLAY_RENDER_COMMAND_TYPE_RECTANGLE:
        break;

      case CLAY_RENDER_COMMAND_TYPE_BORDER:
        TraceLog(LOG_INFO, "Border width (%d, %d, %d, %d) within box (%f, %f) %fx%f", command->renderData.border.width.top, command->renderData.border.width.right, command->renderData.border.width.bottom, command->renderData.border.width.left, command->boundingBox.x, command->boundingBox.y, command->boundingBox.width, command->boundingBox.height);
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
      CLAY_TEXT(clay_string(label));
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

  CLAY(CLAY_SID(clay_string(label)), {
    .layout = {
      .sizing = clay_container_sizing(),
      .layoutDirection = CLAY_LEFT_TO_RIGHT,
      .childGap = 20,
      .padding = CLAY_PADDING_ALL(15),
    },
    .custom = { .customData = checkbox }
  }) {
    checkbox->checkbox.hovered = Clay_Hovered();

    CLAY(CLAY_ID("check"), {
      .layout = {
        .sizing = { .width = CLAY_SIZING_FIXED(50), .height = CLAY_SIZING_FIXED(50) },
      },
      .border = { .width = { 2, 2, 2, 2, 0 } },
      .custom = { .customData = checkbox }
    });

    CLAY_TEXT(clay_string(label), { .userData = checkbox });
  }
}

#include "raylib.h"
#define CLAY_IMPLEMENTATION
#define RAYGUI_IMPLEMENTATION

#include "clay.h"
#include "raygui.h"
#include "cyber/style_cyber.h"
#include <stdlib.h>

Clay_Context *clay;

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

static Color clay_color(Clay_Color color) {
  return (Color) { color.r, color.g, color.b, color.a };
}

static Clay_Dimensions current_screen_size() {
  return (Clay_Dimensions){ .height = GetScreenHeight(), .width = GetScreenWidth() };
}

void ui_initialize() {
  uint32_t memory_size = Clay_MinMemorySize();
  Clay_Arena arena = Clay_CreateArenaWithCapacityAndMemory(memory_size, malloc(memory_size));

  clay = Clay_Initialize(arena,
    current_screen_size(),
    (Clay_ErrorHandler) { .errorHandlerFunction = clay_error_handler, .userData = NULL });

  GuiLoadStyleCyber();
}

void ui_teardown() {

}

void ui_begin() {
  Clay_SetLayoutDimensions(current_screen_size());

  Vector2 mouse_pos = GetMousePosition();
  Clay_SetPointerState((Clay_Vector2) { mouse_pos.x, mouse_pos.y }, IsMouseButtonPressed(MOUSE_LEFT_BUTTON));

  Clay_BeginLayout();
}

void ui_end(float dt) {
  Clay_RenderCommandArray commands = Clay_EndLayout(dt);
  for (int32_t i = 0; i < commands.length; ++i) {
    Clay_RenderCommand *command = &commands.internalArray[i];

    switch(command->commandType) {
      case CLAY_RENDER_COMMAND_TYPE_RECTANGLE:
        GuiDrawRectangle(clay_rect(command->boundingBox), command->renderData.border.width.left, clay_color(command->renderData.border.color), clay_color(command->renderData.rectangle.backgroundColor));
        break;

      default:
        TraceLog(LOG_INFO, "Clay render command: %d", command->commandType);
        break;
    }
  }
}

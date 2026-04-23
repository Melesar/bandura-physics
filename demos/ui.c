#define CLAY_IMPLEMENTATION
#define RAYGUI_IMPLEMENTATION

#include "core.h"
#include "raygui.h"
#include "cyber/style_cyber.h"

#include <float.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

typedef enum {
  ELEMENT_INPUT_INT,
  ELEMENT_INPUT_FLOAT,
} custom_element_type;

typedef struct {
  custom_element_type type;

  union {
    struct {
      int *value;
      int min_value;
      int max_value;
    } input_int;

    struct {
      float *value;
      float min_value;
      float max_value;
    } input_float;
  };

  GuiState state;
} custom_element;

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

static Rectangle clay_rect(Clay_BoundingBox bb) { return (Rectangle){bb.x, bb.y, bb.width, bb.height}; }

static Clay_Dimensions clay_screen_dimensions() {
  return (Clay_Dimensions){.height = GetScreenHeight(), .width = GetScreenWidth()};
}

static Clay_String clay_from_string(const char *label) {
  return (Clay_String){.chars = label, .length = strlen(label), .isStaticallyAllocated = true};
}

static Clay_String clay_icon_string(GuiIconName icon) {
  const char *s = GuiIconText(icon, NULL);
  uint32_t len = strlen(s);
  char *buffer = arena_alloc(memory_size_aligned(len + 1));
  strncpy(buffer, s, len);
  buffer[len] = 0;

  return (Clay_String){.chars = buffer, .length = len, .isStaticallyAllocated = false};
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

  return (Clay_String){.chars = s, .length = len_a + len_b + 1, .isStaticallyAllocated = false};
}

static Clay_Color clay_element_color(int control, int property_base, GuiState state) {
  Color color = GetColor(GuiGetStyle(control, property_base + 3 * state));

  return (Clay_Color){color.r, color.g, color.b, color.a};
}

static Color clay_color_to_ray(Clay_Color color) { return (Color){color.r, color.g, color.b, color.a}; }

static Clay_Color clay_color_from_ray(Color color) { return (Clay_Color){color.r, color.g, color.b, color.a}; }

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

  return (Clay_Dimensions){
      .width = GuiGetTextWidth(string),
      .height = GuiGetStyle(DEFAULT, TEXT_SIZE),
  };
}

static Clay_Sizing clay_container_sizing() { return (Clay_Sizing){.width = CLAY_SIZING_GROW(100, 300)}; }

static void render_custom_element(Clay_RenderCommand *command) {
  custom_element *element = command->renderData.custom.customData;
  Rectangle rect = clay_rect(command->boundingBox);
  bool edit;

  switch (element->type) {
    case ELEMENT_INPUT_INT:
      edit = element->state == STATE_FOCUSED || element->state == STATE_PRESSED;
      GuiValueBox(rect, NULL, element->input_int.value, element->input_int.min_value, element->input_int.max_value,
                  edit);
      break;

    case ELEMENT_INPUT_FLOAT:
      GuiSlider(rect, NULL, NULL, element->input_float.value, element->input_float.min_value,
                element->input_float.max_value);
      break;
  }
}

void ui_initialize() {
  uint32_t memory_size = Clay_MinMemorySize();
  clay_memory = malloc(memory_size);
  Clay_Arena arena = Clay_CreateArenaWithCapacityAndMemory(memory_size, clay_memory);
  Clay_Initialize(arena, clay_screen_dimensions(),
                  (Clay_ErrorHandler){.errorHandlerFunction = clay_error_handler, .userData = NULL});
  Clay_SetMeasureTextFunction(measure_text, NULL);

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
  Clay_SetPointerState((Clay_Vector2){mouse_pos.x, mouse_pos.y}, IsMouseButtonDown(MOUSE_LEFT_BUTTON));
  Clay_UpdateScrollContainers(true, (Clay_Vector2){scroll_wheel.x, scroll_wheel.y}, 0);

  Clay_BeginLayout();
}

void ui_end(float dt) {
  Clay_RenderCommandArray commands = Clay_EndLayout(dt);
  for (int32_t i = 0; i < commands.length; ++i) {
    Clay_RenderCommand *command = &commands.internalArray[i];
    Rectangle rect = clay_rect(command->boundingBox);

    char *string;
    switch (command->commandType) {
      case CLAY_RENDER_COMMAND_TYPE_TEXT:
        string = clay_to_string(command->renderData.text.stringContents);
        GuiDrawText(string, rect, TEXT_ALIGN_LEFT, clay_color_to_ray(command->renderData.text.textColor));
        break;

      case CLAY_RENDER_COMMAND_TYPE_RECTANGLE:
        GuiDrawRectangle(rect, 0, BLANK, clay_color_to_ray(command->renderData.border.color));
        break;

      case CLAY_RENDER_COMMAND_TYPE_BORDER:
        GuiDrawRectangle(rect, command->renderData.border.width.left,
                         clay_color_to_ray(command->renderData.border.color), BLANK);
        break;

      case CLAY_RENDER_COMMAND_TYPE_SCISSOR_START:
        BeginScissorMode((int)roundf(rect.x), (int)roundf(rect.y), (int)roundf(rect.width), (int)roundf(rect.height));
        break;

      case CLAY_RENDER_COMMAND_TYPE_SCISSOR_END:
        EndScissorMode();
        break;

      case CLAY_RENDER_COMMAND_TYPE_CUSTOM:
        render_custom_element(command);
        break;

      default:
        TraceLog(LOG_WARNING, "Unknown Clay render command: %d", command->commandType);
        break;
    }
  }

  custom_arena.pointer = 0;
}

void ui_set_debug(bool is_debug) { Clay_SetDebugModeEnabled(is_debug); }

bool ui_begin_area(const char *title, bool *collapsed) {
  Clay_String s = clay_from_string(title);

  Clay__OpenElementWithId(CLAY_SID(s));
  Clay__ConfigureOpenElement((Clay_ElementDeclaration){
      .layout =
          {
              .sizing = {.width = CLAY_SIZING_FIT(300, FLT_MAX), .height = CLAY_SIZING_FIT(0, FLT_MAX)},
              .layoutDirection = CLAY_TOP_TO_BOTTOM,
          },
      .backgroundColor = clay_element_color(DEFAULT, BACKGROUND_COLOR, STATE_NORMAL),
      .border = {.width = {1, 1, 1, 1, 0}, .color = clay_element_color(DEFAULT, LINE_COLOR, STATE_NORMAL)}});

  CLAY(CLAY_SID(clay_string_concat(title, "statusbar")),
       {
           .layout = {.sizing = {.height = CLAY_SIZING_FIXED(20), .width = CLAY_SIZING_GROW()},
                      .padding = {.left = 10, .right = 2},
                      .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
           .backgroundColor = clay_element_color(STATUSBAR, BASE, STATE_NORMAL),
       }) {
    CLAY(CLAY_SID(clay_string_concat(title, "text")), {.layout = {.sizing = {.width = CLAY_SIZING_GROW(0)}}}) {
      CLAY_TEXT(s, {.textColor = clay_element_color(STATUSBAR, TEXT, STATE_NORMAL)});
    }

    CLAY(CLAY_SID(clay_string_concat(title, "close_btn")),
         {
             .layout =
                 {
                     .sizing = {.width = CLAY_SIZING_FIXED(18), .height = CLAY_SIZING_FIXED(18)},
                     .childAlignment = {.y = CLAY_ALIGN_Y_CENTER, .x = CLAY_ALIGN_X_CENTER},
                 },
             .backgroundColor = clay_element_color(BUTTON, BASE, clay_gui_state()),
         }) {
      if (clay_is_clicked()) {
        *collapsed = !*collapsed;
      }

      CLAY_TEXT(clay_icon_string(ICON_BOX_MINUS_FILL), {.textColor = clay_element_color(BUTTON, TEXT, clay_gui_state()),
                                                        .textAlignment = CLAY_TEXT_ALIGN_CENTER});
    };
  }

  float vertical_padding = *collapsed ? 0 : 15;
  Clay__OpenElementWithId(CLAY_SID(clay_string_concat(title, "content")));
  Clay__ConfigureOpenElement((Clay_ElementDeclaration){
      .layout =
          {
              .sizing = {.width = CLAY_SIZING_GROW()},
              .layoutDirection = CLAY_TOP_TO_BOTTOM,
              .childGap = 10,
              .padding = {.left = 10, .right = 10, .top = vertical_padding, .bottom = vertical_padding},
          },
  });

  return !*collapsed;
}

void ui_end_area() {
  Clay__CloseElement(); // Children container
  Clay__CloseElement(); // Modal window
}

void ui_label(char *label) {
  CLAY_TEXT(clay_from_string(label), {.textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL)});
}

static void ui_value_label(const char *label, Clay_String value) {
  CLAY(CLAY_SID(clay_string_concat(label, "container")), {
                                                             .layout =
                                                                 {
                                                                     .sizing = clay_container_sizing(),
                                                                     .layoutDirection = CLAY_LEFT_TO_RIGHT,
                                                                 },
                                                         }) {
    CLAY(CLAY_SID(clay_string_concat(label, "label")), {.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
      CLAY_TEXT(clay_from_string(label), {.textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL)});
    };

    CLAY(CLAY_SID(clay_string_concat(label, "value_container")), {.layout = {
                                                                      .childAlignment = {.x = CLAY_ALIGN_X_RIGHT},
                                                                      .sizing = {.width = CLAY_SIZING_GROW()},
                                                                  }}) {
      CLAY_TEXT(value, {.textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL)});
    }
  }
}

static void ui_prefix_label(const char *label) {
  CLAY(CLAY_SID(clay_string_concat(label, "label")), {.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
    CLAY_TEXT(clay_from_string(label), {.textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL)});
  };
}

void ui_label_v3(const char *label, v3 value) {
  char *v = arena_alloc(80); // Allocate more to keep the arena 8-bytes aligned
  snprintf(v, 80, "(%.2f, %.2f, %.2f)", value.x, value.y, value.z);
  Clay_String vs = {.chars = v, .length = strlen(v), .isStaticallyAllocated = false};

  ui_value_label(label, vs);
}

void ui_label_bool(const char *label, bool value) {
  Clay_String s = clay_from_string(value ? "true" : "false");
  ui_value_label(label, s);
}

void ui_label_float(char *label, float value) {
  char *v = arena_alloc(80);
  snprintf(v, 80, "%.2f", value);
  Clay_String vs = {.chars = v, .length = strlen(v), .isStaticallyAllocated = false};

  ui_value_label(label, vs);
}

void ui_label_string(char *label, char *value) { ui_value_label(label, clay_from_string(value)); }

void ui_label_int(const char *label, count_t value) {
  char *v = arena_alloc(80);
  snprintf(v, 80, "%d", value);
  Clay_String vs = {.chars = v, .length = strlen(v), .isStaticallyAllocated = false};

  ui_value_label(label, vs);
}

void ui_label_matrix(const char *label, m3 value) {
  CLAY(CLAY_SID(clay_string_concat(label, "container")), {.layout = {.sizing = clay_container_sizing()}}) {
    CLAY(CLAY_SID(clay_string_concat(label, "label")), {.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}}) {
      CLAY_TEXT(clay_from_string(label), {.textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL)});
    };

    CLAY(CLAY_SID(clay_string_concat(label, "matrix")),
         {.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .padding = CLAY_PADDING_ALL(3), .childGap = 5}}) {
      float *rows[] = {&value.m0[0], &value.m1[0], &value.m2[0]};
      for (count_t i = 0; i < 3; ++i) {
        CLAY(CLAY_SIDI(clay_string_concat(label, "row"), i + 1), {.layout = {.childGap = 5}}) {
          for (count_t j = 0; j < 3; ++j) {
            char *s = arena_alloc(32);
            snprintf(s, 32, "%.2f", rows[i][j]);

            CLAY_TEXT(clay_from_string(s), {.textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL)});
          }
        }
      }
    }
  }
}

void ui_checkbox(const char *label, bool *is_checked) {
  CLAY(CLAY_SID(clay_string_concat(label, "checkbox")), {
                                                            .layout = {.sizing = clay_container_sizing(),
                                                                       .layoutDirection = CLAY_LEFT_TO_RIGHT,
                                                                       .childGap = 10,
                                                                       .childAlignment = {.y = CLAY_ALIGN_Y_CENTER}},
                                                        }) {
    if (clay_is_clicked()) {
      *is_checked = !*is_checked;
    }

    GuiState state = clay_gui_state();
    float text_size = 1.2 * GuiGetStyle(DEFAULT, TEXT_SIZE);

    CLAY(CLAY_SID(clay_string_concat(label, "check")),
         {
             .layout =
                 {
                     .sizing = {.width = CLAY_SIZING_FIXED(text_size), .height = CLAY_SIZING_FIXED(text_size)},
                 },
             .border = {.width = {2, 2, 2, 2, 0}, .color = clay_element_color(CHECKBOX, BORDER, state)},
             .backgroundColor = clay_element_color(CHECKBOX, TEXT, state),
         }) {
      if (*is_checked)
        CLAY_TEXT(clay_icon_string(ICON_BOX_CIRCLE_MASK), {.textColor = clay_element_color(LABEL, TEXT, STATE_DISABLED),
                                                           .textAlignment = CLAY_TEXT_ALIGN_CENTER});
    };

    CLAY_TEXT(clay_from_string(label), {.textColor = clay_element_color(CHECKBOX, TEXT, state)});
  }
}

bool ui_dropdown(const char *label, char **values, count_t values_count, count_t *selected, bool *active) {
  bool result = false;
  CLAY(CLAY_SID(clay_string_concat(label, "dropdown")), {.layout = {
                                                             .sizing = clay_container_sizing(),
                                                             .childGap = 5,
                                                         }}) {
    CLAY_TEXT(clay_from_string(label), {.textColor = clay_element_color(DROPDOWNBOX, TEXT, clay_gui_state())});

    CLAY_AUTO_ID({.layout = {.sizing = {.width = CLAY_SIZING_GROW()}}});

    CLAY(CLAY_SID(clay_string_concat(label, "dropdown_box")),
         {.border = {.width = {1, 1, 1, 1, 0}, .color = clay_element_color(DROPDOWNBOX, BORDER, clay_gui_state())},
          .backgroundColor = clay_element_color(DROPDOWNBOX, BASE, clay_gui_state())}) {
      if (!*active && clay_is_clicked()) {
        *active = true;
      }
      char *text = values[*selected];
      CLAY(CLAY_SID(clay_string_concat(label, "dropdown_text")),
           {.layout = {.sizing = {.width = CLAY_SIZING_FIT(120, INT_MAX)},
                       .childAlignment = {.x = CLAY_ALIGN_X_CENTER}}}) {

        CLAY_TEXT(clay_from_string(text), {.textColor = clay_element_color(DROPDOWNBOX, TEXT, clay_gui_state()),
                                           .textAlignment = CLAY_TEXT_ALIGN_CENTER});
      }

      if (*active) {
        CLAY(CLAY_SID(clay_string_concat(label, "dropdown_options")),
             {.floating = {.attachTo = CLAY_ATTACH_TO_PARENT,
                           .attachPoints = {.parent = CLAY_ATTACH_POINT_LEFT_BOTTOM,
                                            .element = CLAY_ATTACH_POINT_LEFT_TOP},
                           .offset = {.y = 5}},
              .border = {.width = {1, 1, 1, 1, 1}, .color = clay_element_color(DROPDOWNBOX, BORDER, STATE_NORMAL)},
              .layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM, .sizing = {.width = CLAY_SIZING_FIT(120, INT_MAX)}}}) {
          for (count_t i = 0; i < values_count; ++i) {
            CLAY(CLAY_SIDI(clay_string_concat(label, "dropdown_item"), i),
                 {.layout = {.childAlignment = {.x = CLAY_ALIGN_X_CENTER}, .sizing = clay_container_sizing()},
                  .backgroundColor = clay_element_color(DROPDOWNBOX, BASE, clay_gui_state())}) {
              if (clay_is_clicked()) {
                *active = false;
                *selected = i;
                result = true;
              }
              CLAY_TEXT(clay_from_string(values[i]),
                        {.textColor = clay_element_color(DROPDOWNBOX, TEXT, clay_gui_state()),
                         .textAlignment = CLAY_TEXT_ALIGN_CENTER});
            }
          }
        }
      }
    }

    CLAY_AUTO_ID() {
      if (clay_is_clicked()) {
        *active = !*active;
      }
      CLAY_TEXT(clay_icon_string(ICON_ARROW_DOWN),
                {.textColor = clay_element_color(DROPDOWNBOX, TEXT, clay_gui_state())});
    }
  }
  return result;
}

void ui_value_int(const char *label, int *value, int min_value, int max_value) {
  custom_element *element = arena_alloc(sizeof(custom_element));
  element->type = ELEMENT_INPUT_INT;
  element->input_int.value = value;
  element->input_int.min_value = min_value;
  element->input_int.max_value = max_value;

  CLAY(CLAY_SID(clay_string_concat(label, "container")), {
                                                             .layout =
                                                                 {
                                                                     .sizing = clay_container_sizing(),
                                                                     .layoutDirection = CLAY_LEFT_TO_RIGHT,
                                                                 },
                                                         }) {

    ui_prefix_label(label);

    element->state = clay_gui_state();

    CLAY(CLAY_SID(clay_string_concat(label, "value_container")),
         {
             .layout =
                 {
                     .childAlignment = {.x = CLAY_ALIGN_X_RIGHT},
                     .sizing = {.width = CLAY_SIZING_GROW(),
                                .height = CLAY_SIZING_FIT(GuiGetStyle(DEFAULT, TEXT_SIZE), FLT_MAX)},
                 },
             .custom = {.customData = element},
         });
  }
}

void ui_value_float(const char *label, float *value, float min_value, float max_value) {
  custom_element *element = arena_alloc(sizeof(custom_element));
  element->type = ELEMENT_INPUT_FLOAT;
  element->input_float.value = value;
  element->input_float.min_value = 0.0;
  element->input_float.max_value = 1.0;

  CLAY(CLAY_SID(clay_string_concat(label, "container")),
       {
           .layout = {.sizing = clay_container_sizing(), .layoutDirection = CLAY_LEFT_TO_RIGHT, .childGap = 5},
       }) {

    ui_prefix_label(label);

    element->state = clay_gui_state();

    CLAY(CLAY_SID(clay_string_concat(label, "value_container")),
         {
             .layout =
                 {
                     .childAlignment = {.x = CLAY_ALIGN_X_RIGHT},
                     .sizing = {.width = CLAY_SIZING_GROW(),
                                .height = CLAY_SIZING_FIT(GuiGetStyle(DEFAULT, TEXT_SIZE), FLT_MAX)},
                 },
             .custom = {.customData = element},
         });

    char *value_text = arena_alloc(128);
    snprintf(value_text, 128, "%.2f", *value);
    CLAY_TEXT(clay_from_string(value_text), {
                                                .textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL),
                                            });
  }
}

void ui_label_stat(const char *label, float value) {
  Clay_Color background = clay_element_color(DEFAULT, BACKGROUND_COLOR, STATE_NORMAL);
  background.a = 150;

  CLAY(CLAY_SID(clay_string_concat(label, "stat_container")),
       {
           .layout =
               {
                   .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER},
                   .padding = CLAY_PADDING_ALL(5),
                   .childGap = 5,
               },
           .backgroundColor = background,
       }) {
    CLAY_TEXT(clay_from_string(label), {.textColor = clay_element_color(LABEL, TEXT, STATE_NORMAL)});

    char *value_text = arena_alloc(16);
    snprintf(value_text, 16, "%d", (int)value);

    CLAY_TEXT(clay_from_string(value_text), {.textColor = clay_color_from_ray(LIME)});
  }
}

bool ui_button(const char *text) {
  bool clicked = false;
  CLAY(CLAY_SID(clay_string_concat("button", text)),
       {.backgroundColor = clay_element_color(BUTTON, BASE, clay_gui_state()),
        .border = {.width = {.top = 1, .bottom = 1, .left = 1, .right = 1},
                   .color = clay_element_color(BUTTON, BORDER, clay_gui_state())},
        .layout = {.padding = {.top = 5, .bottom = 5, .left = 10, .right = 10}}}) {
    CLAY_TEXT(clay_from_string(text), {.textColor = clay_element_color(BUTTON, TEXT, clay_gui_state())});
    clicked = clay_is_clicked();
  }

  return clicked;
}

Clay_Color ui_text_color(int control) { return clay_element_color(control, TEXT, clay_gui_state()); }

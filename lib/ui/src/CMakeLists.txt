SET(SOURCES screens/ui_MainScreen.c
    screens/ui_TraceScreen.c
    screens/ui_TopScreen.c
    screens/ui_LoadScreen.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.cpp)

add_library(ui ${SOURCES})

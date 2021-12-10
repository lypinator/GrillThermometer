// ux_manager.c


#include "main.h"
#include "ux_manager.h"
#include <stdio.h>

// private defines
#define TOTAL_RECIPES 3

// Global Constants


// Modular Constants


// global variables

extern uint8_t beepActive;

uint8_t COOKING = false;

// Screens
ui_screen currentScreen;
ui_screen lastScreen;
ui_screen screenNumber;
extern uint8_t Timer;

Cooking_Item ITEM_1 = {
    .Item_Name = {"%s", "!!!!", 0, 0, true, "Chicken-165"},
    .Final_Temp = 165
};
Cooking_Item ITEM_2 = {
    .Item_Name = {"%s", "!!!!", 0, 0, true, " Steak-155 "},
    .Final_Temp = 155
};
Cooking_Item ITEM_3 = {
    .Item_Name = {"%s", "!!!!", 0, 0, true, " Pork-150  "},
    .Final_Temp = 150
};

Cooking_Item Recipe_Book[TOTAL_RECIPES];
Cooking_Item Current_Recipe = {0};
uint8_t Recipe_ID = 0;

char message[25] = "";

// Display-wrapped values
// format seq (numeric): {<format string>, <error message>, <Xpos>, <Ypos>, <valid?>, <init value>}
DWfloat counter = {"%5.2f", "----", 0, 0, true, 0};
DWfloat tempInF = {"%4.1f", "----", 0, 0, true, 72.3};
DWint16_t Meat_F = {"%3d", "!!!!", 0,0, true, 0};
DWint16_t AMB_F = {"%3d", "!!!!", 0,0, true, 0};

DWint8_t Seconds = {"%2d", "----", 0, 0, true, 0};
DWint8_t Minutes = {"%2d", "----", 0, 0, true, 0};
DWint8_t Hours   = {"%2d", "----", 0, 0, true, 0};


// modular variables


// module prototypes



// ***************
// Start Of Code
// ***************
// Screen switching utility that manages pre-, post-, and screen switch conditions
void SwitchScreens(ui_screen screen_no)
{
  lastScreen = currentScreen;

  
#pragma diag_suppress= Pa149
  // what must be done before current screen is switched out
  switch (lastScreen) {
  }
  
  
  // what must be done before screen is switched in
  switch (screen_no) {
  }
#pragma diag_warning= Pa149
  
  // Switch the screens
  switch (screen_no) {
    case STARTUP_SCREEN:
      SSD1306_GotoXY (0,0);
      SSD1306_Puts ("MEAT BUDDY", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 30);
      SSD1306_Puts ("I cook steak well-", &Font_7x10, SSD1306_COLOR_WHITE);
      //SSD1306_Puts ("Salmonella is bad.", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 50);
      SSD1306_Puts ("  done. Sue me.  ", &Font_7x10, SSD1306_COLOR_WHITE);
      break;
    case MAIN:
      // clear the screen from the previos dispayed data
      SSD1306_Clear();
      // Put up the "persistant" info (like data labels)
      SSD1306_GotoXY (0,0);
      SSD1306_Puts ("MEAT BUDDY", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 30);
      SSD1306_Puts ("Pre-Programmed =>", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 50);
      //SSD1306_Puts ("<= Custom Cook", &Font_7x10, SSD1306_COLOR_WHITE);

      tempInF.xPos = 55;
      tempInF.yPos = 30;
      break;
    case ITEM_LIST:
      Recipe_Book[0] = ITEM_1;
      Recipe_Book[1] = ITEM_2;
      Recipe_Book[2] = ITEM_3;
      SSD1306_GotoXY (0,0);
      SSD1306_Puts ("MEAT SELECT", &Font_11x18, SSD1306_COLOR_WHITE);
      if(Recipe_ID == 0){
        SSD1306_GotoXY (0, 50);
        SSD1306_Puts ("           Next =>", &Font_7x10, SSD1306_COLOR_WHITE);
      }
      else if (Recipe_ID == (TOTAL_RECIPES-1)){
        SSD1306_GotoXY (0, 50);
        SSD1306_Puts ("<= Prev           ", &Font_7x10, SSD1306_COLOR_WHITE);
      }
      else{
        SSD1306_GotoXY (0, 50);
        SSD1306_Puts ("<= Prev    Next =>", &Font_7x10, SSD1306_COLOR_WHITE);
      }
      break;
    case CUSTOM_COOK:
      break;
     //18 letter max
    case COOKING_READY:
      SSD1306_Clear();
      SSD1306_Clear();
      SSD1306_GotoXY (0,0);
      SSD1306_Puts ("Delizioso!", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 30);
      SSD1306_Puts ("Press Any Button", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 50);
      SSD1306_Puts ("To Begin Recording", &Font_7x10, SSD1306_COLOR_WHITE);
      break;
    case COOKING_TEMP:
      SSD1306_Clear();
      SSD1306_GotoXY (0,0);
      SSD1306_Puts (" Meat Temp ", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 30);
      SSD1306_Puts ("MEAT: ", &Font_7x10, SSD1306_COLOR_WHITE);
      Meat_F.xPos = 32;
      Meat_F.yPos = 30;
      SSD1306_GotoXY (75, 30);
      SSD1306_Puts ("AMB: ", &Font_7x10, SSD1306_COLOR_WHITE);
      AMB_F.xPos = 100;
      AMB_F.yPos = 30;
      SSD1306_GotoXY (0, 50);
      SSD1306_Puts ("<= TIME    PROG =>", &Font_7x10, SSD1306_COLOR_WHITE);
      break;
     case TIME_COOKED:
      SSD1306_Clear();
      SSD1306_GotoXY (0,0);
      SSD1306_Puts (" Cook Time ", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 25);
      SSD1306_Puts ("00:00:00", &Font_11x18, SSD1306_COLOR_WHITE);
      Seconds.xPos = 66;
      Seconds.yPos = 25;
      Minutes.xPos = 33;
      Minutes.yPos = 25;
      Hours.xPos = 0;
      Hours.yPos = 25;
      SSD1306_GotoXY (0, 50);
      SSD1306_Puts ("<= PROG    TEMP =>", &Font_7x10, SSD1306_COLOR_WHITE);
      break;
     case COOKING_PROGRESS:
      SSD1306_Clear();
      SSD1306_GotoXY (0,0);
      SSD1306_Puts (" Progress ", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_DrawRectangle(5, 25, 140, 20, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 50);
      SSD1306_Puts ("<= TEMP    TIME =>", &Font_7x10, SSD1306_COLOR_WHITE);
      break;
      break;
     case CANCEL_CONFIRM:
      SSD1306_Clear();
      SSD1306_GotoXY (0,0);
      SSD1306_Puts ("Cancel Cook", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 30);
      SSD1306_Puts ("Are you sure you", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 50);
      SSD1306_Puts ("want to cancel?", &Font_7x10, SSD1306_COLOR_WHITE);
      break;
     case COOKING_COMPLETE:
      beepActive = true;
      SSD1306_Clear();
      SSD1306_GotoXY (0,0);
      SSD1306_Puts (" Meat Cooked ", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 30);
      SSD1306_Puts (" Press any button to ", &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (0, 50);
      SSD1306_Puts ("   cancel alarm   ", &Font_7x10, SSD1306_COLOR_WHITE);
      break;
    default:
      break;
  }
/*
  case MESSAGE:
    SSD1306_Clear();
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Message", &Font_11x18, SSD1306_COLOR_WHITE);
    count.xPos = 60;
    count.yPos = 40;
    break;
  }*/
  currentScreen = screen_no;
  SSD1306_UpdateScreen();
  //Timer = 0;
  
#pragma diag_suppress= Pa149
  // what must be done after screen is switched in
  switch (currentScreen) {
  }
#pragma diag_warning= Pa149
  
}


//// Keyboard Processor
//
//uint8_t ProcessKeyCode (uint16_t key_code)
//{
//  switch (key_code) {
//  case 0:
//    break;
//  case 1:
//    break;
//  case 2:
//    break;
//  case 3:
//    break;
//  }
//  
//  return true;
//}


// context sensitive keyboard processor
//0x0E: LEFT
//0x0D: SELECT
//0x0B: RIGHT
//0x07: BACK
uint8_t ProcessKeyCodeInContext (uint16_t key_code)
{
  switch (currentScreen) {
  case  MAIN:
    switch (key_code) {
    case 0x0E:
      //SwitchScreens(CUSTOM_COOK);
      break;
    case 0x0D:
      break;
    case 0x0B:
      SSD1306_Clear();
      Current_Recipe = ITEM_1;
      SwitchScreens(ITEM_LIST);
      break;
    case 0x07:
      SwitchScreens(MAIN);
      break;
    }
    break;
  case  ITEM_LIST:
    switch (key_code) {
    case 0x0E:
      if(Recipe_ID){
        Recipe_ID--;
        Current_Recipe = Recipe_Book[Recipe_ID];
        SwitchScreens(ITEM_LIST);
      }
      break;
    case 0x0D:
      SwitchScreens(COOKING_READY);
      break;
    case 0x0B:
      if(Recipe_ID != (TOTAL_RECIPES-1)){
        Recipe_ID++;
        Current_Recipe = Recipe_Book[Recipe_ID];
        SwitchScreens(ITEM_LIST);
      }
      break;
    case 0x07:
      SwitchScreens(MAIN);
      break;
    }
    break;
   case  CUSTOM_COOK:
    switch (key_code) {
    case 0x0E:
      //[INTENTIONALLY EMPTY]
      break;
    case 0x0D:
      //[CUSTOM COOK LOGIC]
      break;
    case 0x0B:
      SwitchScreens(MAIN);
      break;
    case 0x07:
      SwitchScreens(MAIN);
      break;
    }
    break;
   case  COOKING_READY:
    switch (key_code) {
    case 0x0E:
    case 0x0D:
    case 0x0B:
      COOKING = true;
      START_COOKING_TIMER
      SwitchScreens(COOKING_TEMP);
      break;
    case 0x07:
      SwitchScreens(MAIN);
      break;
    }
    break;
   case  COOKING_TEMP:
    switch (key_code) {
    case 0x0E:
      SwitchScreens(TIME_COOKED);
      break;
    case 0x0D:
      SwitchScreens(COOKING_TEMP);
      break;
    case 0x0B:
      SwitchScreens(COOKING_PROGRESS);
      break;
    case 0x07:
      SwitchScreens(CANCEL_CONFIRM);
      break;
    }
    break;
   case  TIME_COOKED:
    switch (key_code) {
    case 0x0E:
      SwitchScreens(COOKING_PROGRESS);
      break;
    case 0x0D:
      SwitchScreens(COOKING_TEMP);
      break;
    case 0x0B:
      SwitchScreens(COOKING_TEMP);
      break;
    case 0x07:
      SwitchScreens(CANCEL_CONFIRM);
      break;
    }
    break;
   case  COOKING_PROGRESS:
    switch (key_code) {
    case 0x0E:
      SwitchScreens(COOKING_TEMP);
      break;
    case 0x0D:
      SwitchScreens(COOKING_TEMP);
      break;
    case 0x0B:
      SwitchScreens(TIME_COOKED);
      break;
    case 0x07:
      SwitchScreens(CANCEL_CONFIRM);
      break;
    }
    break;
   case  CANCEL_CONFIRM:
    switch (key_code) {
    case 0x0E:
    case 0x0D:
    case 0x0B:
      STOP_COOKING_TIMER
      SwitchScreens(COOKING_TEMP);
      break;
    case 0x07:
      COOKING = false;
      SwitchScreens(MAIN);
      break;
    }
    break;
   case COOKING_COMPLETE:
     beepActive = false;
     STOP_COOKING_TIMER
     COOKING = false;
     SwitchScreens(MAIN);
     break;
  }
  
  return true;
}



void UpdateScreenValues(void)
{
  if(COOKING && Meat_F.data >= (Current_Recipe.Final_Temp - 3)) {
    SwitchScreens(COOKING_COMPLETE);
  }
  float progress;
  char displayString[25];
    switch (currentScreen) {
    case STARTUP_SCREEN:
      break;
    case MAIN:
      break;
    case ITEM_LIST:
      SSD1306_GotoXY(0,25);
      sprintf(displayString, Current_Recipe.Item_Name.format, Current_Recipe.Item_Name.data);
      SSD1306_Puts(Current_Recipe.Item_Name.data, &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen(); 
      break;
    case CUSTOM_COOK:
      break;
    case COOKING_READY:
      break;
    case COOKING_TEMP:
      sprintf(displayString, Meat_F.format, Meat_F.data);
      SSD1306_GotoXY(Meat_F.xPos,Meat_F.yPos);
      SSD1306_Puts(displayString, &Font_7x10, SSD1306_COLOR_WHITE);
      sprintf(displayString, AMB_F.format, AMB_F.data);
      SSD1306_GotoXY(AMB_F.xPos,AMB_F.yPos);
      SSD1306_Puts(displayString, &Font_7x10, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen(); 
      break;
    case TIME_COOKED:
      Seconds.data =  (timerValue % 60);
      Minutes.data = ((timerValue /60)%60);
      Hours.data = (timerValue /3600);
      Seconds.xPos = 66;
      Seconds.yPos = 25;
      Minutes.xPos = 33;
      Minutes.yPos = 25;
      Hours.xPos = 0;
      Hours.yPos = 25;
      SSD1306_GotoXY(Seconds.xPos,Seconds.yPos);
      sprintf(displayString, Seconds.format, Seconds.data);
      SSD1306_Puts (displayString, &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(Minutes.xPos,Minutes.yPos);
      sprintf(displayString, Minutes.format, Minutes.data);
      SSD1306_Puts (displayString, &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY(Hours.xPos,Hours.yPos);
      sprintf(displayString, Hours.format, Hours.data);
      SSD1306_Puts (displayString, &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen(); 
      break;
    case COOKING_PROGRESS:
      progress = (float)Meat_F.data/(float)Current_Recipe.Final_Temp;
      if (progress < 0) progress = 0;
      SSD1306_DrawFilledRectangle(5, 25, (uint16_t)(progress*140), 20, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen(); 
      break;
    case CANCEL_CONFIRM:
      break;
    }
    //SSD1306_UpdateScreen(); //display
}
  /*
    case SETTINGS:
      SSD1306_GotoXY (0,40);
      SSD1306_Puts("DegF", &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen(); 
      break;
    case MESSAGE:
      SSD1306_GotoXY (0,20);
      SSD1306_Puts(message, &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_GotoXY (count.xPos,count.yPos);
      sprintf(displayString, count.format, count.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
      SSD1306_UpdateScreen(); 
      break;
    }
      if (counter.valid) {
        sprintf(displayString, counter.format, counter.data);
        SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
      }*/
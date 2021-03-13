/*
 * uwcomp.h
 *
 * Created: 04.06.2020 22:39:07
 *  Author: Drumir
 */ 


#ifndef UWCOMP_H_
#define UWCOMP_H_

#define M_MENU    0   // Режим меню
#define M_DEEP    1   // Режим нырка (глубже метра)
#define M_SURFACE 2   // Режим отдыха на поверхности воды
#define M_AIR     3   // Режим на воздухе. Определяется по давлению и по датчику воды
#define M_SLEEP   4   // Режим сна

#define MI_TimeH          0   // Выбранный пункт меню
#define MI_TimeM          1
#define MI_Contrast       2
#define MI_Last           3

#define MENU_MODE_TIME  6       // Длительность режима M_MENU
#define AIR_MODE_TIME   30      // Длительность режима M_AIR
#define PRESS_ARRAY_LENGTH  32   // Размер массива измерений давления 



#endif /* UWCOMP_H_ */

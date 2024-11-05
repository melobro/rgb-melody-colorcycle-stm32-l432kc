#include <stdio.h>



 typedef struct 
{
    int frequency; 
    int value; 
}note;



enum{
NOTE_C3 =131,
NOTE_Cis3=138,
NOTE_D3 =147,
NOTE_Dis3=155,
NOTE_E3=165,
NOTE_F3=174,
NOTE_Fis3=185,
NOTE_G3=196,
NOTE_Gis3=207,
NOTE_A3=220,
NOTE_Ais3=233,
NOTE_H3=247,
NOTE_C4=262,
NOTE_Cis4=277,
NOTE_D4=293,
NOTE_Dis4=311,
NOTE_E4=329,
NOTE_F4=349,
NOTE_Fis4=370,
NOTE_G4=392,
NOTE_Gis4=415,
NOTE_A4=440,
NOTE_Ais4=466,
NOTE_H4=494,
NOTE_C5=523,
NOTE_Cis5=554,
NOTE_D5=587,
NOTE_Dis5=622,
NOTE_E5=659,
NOTE_F5=698,
NOTE_Fis5=740,
NOTE_G5=784,
NOTE_Gis5=830,
NOTE_A5=880,
NOTE_Ais5=932,
NOTE_H5=988, 
Pause = 2000,
}; 
 

const note song2[] = {
 
   {NOTE_Fis4,8}, 
   {NOTE_G4,8},
   {NOTE_A4,8},
   {NOTE_H4,8},
   {NOTE_H4,4},
   {NOTE_H4,8},
   {NOTE_H4,8},
   {NOTE_H4,4},
   {NOTE_H4,4},
   {NOTE_Ais4,4},
   {NOTE_Ais4,8},
   {NOTE_Gis4,16},
   {NOTE_Fis4,4},
   {NOTE_Fis4,8},
   {NOTE_A4,8},
   {NOTE_A4,4},
   {NOTE_A4,4},
   {NOTE_A4,4},
   {NOTE_A4,8},
   {NOTE_Gis4,8},
   {NOTE_Gis4,4},
   {NOTE_Gis4,4},
   {NOTE_Fis4,8},
   {NOTE_H4,8},
   {NOTE_H4,4},
   {NOTE_H4,4},
   {NOTE_D5,4},
   {NOTE_H4,8},
   {NOTE_Ais4,8},
   {NOTE_Ais4,4},
   {NOTE_Ais4,4},
   {NOTE_Cis5,4},
   {NOTE_Cis5,8},
   {NOTE_H4,8},
   {NOTE_Cis5,4},
   {NOTE_H4,4},



};


const note song3 [] = { 

// Zeile 1  
{NOTE_H4, 4},
{NOTE_H4, 8},
{NOTE_D5, 8},
{NOTE_Cis5, 8},
{NOTE_H4, 8},
{NOTE_A4, 4},
{NOTE_H4, 44},
{NOTE_A4, 4},
{NOTE_A4, 8},
{NOTE_G4, 8},
{NOTE_Fis4, 8},
{NOTE_A4, 8},
{NOTE_G4, 4},
{NOTE_G4, 4},
{NOTE_Fis4, 44},
{NOTE_H4, 8},
{NOTE_H4, 44},
{NOTE_D5, 8},
{NOTE_Cis5, 8},
{NOTE_H4, 8},
{NOTE_A4, 4},
{NOTE_H4, 44},
{NOTE_Cis5, 4},
{NOTE_D5, 4},
{NOTE_D5, 4},
{NOTE_E5, 2}, 




// Zeile 2 

{NOTE_Cis5, 44},
{NOTE_D5, 8},
{NOTE_Cis5, 8},
{NOTE_H4, 8},
{NOTE_A4, 8},
{NOTE_Cis5, 8},

{NOTE_D5, 4},
{NOTE_D5, 4},
{NOTE_Cis5, 2},

{NOTE_H4, 44},
{NOTE_D5, 8},
{NOTE_Cis5, 8},
{NOTE_H4, 8},
{NOTE_A4, 8},
{NOTE_Cis5, 8},
{NOTE_D5, 4},
{NOTE_D5, 4},
{NOTE_Cis5, 44},

{NOTE_H4, 44},
{NOTE_D5, 8},
{NOTE_Cis5, 8},
{NOTE_H4, 8},
{NOTE_A4, 4},

{NOTE_H4, 4},
{NOTE_H4, 4},
{NOTE_H4, 2},


{NOTE_H4, 8},
{NOTE_A4, 8},
{NOTE_G4, 4},
{NOTE_Fis4, 8},
{NOTE_A4, 8},

//Zeile 3

{NOTE_G4, 4},
{NOTE_G4, 4},
{NOTE_Fis4, 00},
//Ganze Pause 
{NOTE_Fis4, 4},
{NOTE_A4, 4},
{NOTE_H4, 4},
{NOTE_D5, 4},
{NOTE_D5, 2},
{NOTE_G5, 8},
{NOTE_Fis5, 8},
{NOTE_E5, 8},
{NOTE_G5, 8},
{NOTE_Fis5, 4},
{NOTE_D5, 44},
{NOTE_E5, 4},
{NOTE_A4, 4},

//Zeile 4

{NOTE_A4, 44},
{NOTE_E5, 8},
{NOTE_Fis5, 8},
{NOTE_E5, 8},

};


const note song4[]= {
    {NOTE_Fis4,8},
    {NOTE_D4,8},
    {NOTE_D4,16},
    {NOTE_E4,8},
    {NOTE_F4,16},
    {NOTE_E4,16},
    {NOTE_D4,8},
    {NOTE_Cis4,16},
    {NOTE_D4,16},
    {NOTE_E4,8},
    {NOTE_Fis4,8},
    {NOTE_H4,8},
    {NOTE_H3,16},
    {NOTE_Cis4,8},
    {NOTE_D4,16},
    {NOTE_E4,16},
    {NOTE_D4,8},
    {NOTE_Cis4,16},
    {NOTE_A4,16},
    {NOTE_G4,8},

};



const size_t song2_len = sizeof(song2)/sizeof(song2[0]);
const size_t song3_len = sizeof(song3)/sizeof(song3[0]);
const size_t song4_len = sizeof(song4)/sizeof(song4[0]);


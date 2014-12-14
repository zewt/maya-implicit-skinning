/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
#include <cstdio>
#include <cstdlib>
#include "ppm_loader.hpp"

// =============================================================================
namespace Ppm_loader {
// =============================================================================

bool read(const std::string& imageName, int& width, int& height, int*& data)
{
  FILE* fp;
  //PGM Headers Variable Declaration
  int *ptr;
  int q,i;
  char header[100];
  //Open file for Reading in Binary Mode
  fp = fopen(imageName.c_str(),"rb");
  if(fp == NULL){
        printf("Image does not exist \n");
        return false;
    } else {
        //Check the PGM file Type P2 or P5
        char* dummy = fgets(header,100,fp);
        if((header[0] != 80) || (header[1] != 54 )){
            printf("Image is not PPM\n");
            return false;
        }
        //Check the Comments
        dummy = fgets(header,100,fp);
        while(header[0] == '#')	{
            //printf("%c\n", header[0]);
            dummy = fgets(header,100,fp);
        }
        //Get Width and Height
        width = strtol(header,(char**)(&ptr),0);
        height = atoi((char*)ptr);

        printf ("img width = %d, height = %d\n", width, height);

        // Get Maximum Gray Value
        dummy = fgets(header,100,fp);
        q = strtol(header,(char**)(&ptr),0);
        //Allocating Array Size
        unsigned char* charImage = (unsigned char*) malloc(width*height*3*sizeof(unsigned char));
        data = (int *)malloc(width * height * sizeof(int));

        // Pixel Extraction
        int idummy = fread(charImage,1,(width*height)*3*sizeof(unsigned char),fp);
        for(i=0;i<width*height;i++){
            int r = (int)charImage[i*3];
            int g = (int)charImage[i*3+1];
            int b = (int)charImage[i*3+2];
            data[i]= (b << 16) | (g << 8) | r ;
        }

        // Avoid stupid warnings...
        *dummy  = (char)idummy;
        idummy  = (int)*dummy;
        idummy  = q;

        // Pixel Extraction
        fclose(fp);
        free(charImage);
        return true;
    }
}

// -----------------------------------------------------------------------------

bool read_with_alpha(const std::string& imageName, int& width, int& height, int*& data)
{
  FILE* fp;
  //PGM Headers Variable Declaration
  int *ptr;
  int q,i;
  char header[100];
  //Open file for Reading in Binary Mode
  fp = fopen(imageName.c_str(),"rb");
  if(fp == NULL){
        printf("Image does not exist \n");
        return false;
    } else {
        //Check the PGM file Type P2 or P5
        char* dummy = fgets(header,100,fp);
        if((header[0] != 80) || (header[1] != 54 )){
            printf("Image is not PPM\n");
            return false;
        }
        //Check the Comments
        dummy = fgets(header,100,fp);
        while(header[0] == '#')	{
            //printf("%c\n", header[0]);
            dummy = fgets(header,100,fp);
        }
        //Get Width and Height
        width = strtol(header,(char**)(&ptr),0);
        height = atoi((char*)ptr);

        printf ("img width = %d, height = %d\n", width, height);

        // Get Maximum Gray Value
        dummy = fgets(header,100,fp);
        q = strtol(header,(char**)(&ptr),0);
        //Allocating Array Size
        unsigned char* charImage = (unsigned char*) malloc(width*height*3*sizeof(unsigned char));
        data = (int *)malloc(width * height * sizeof(int));

        // Pixel Extraction
        int idummy = fread(charImage,1,(width*height)*3*sizeof(unsigned char),fp);
        for(i=0;i<width*height;i++){
            int r = (int)charImage[i*3];
            int g = (int)charImage[i*3+1];
            int b = (int)charImage[i*3+2];
            int max = (r > g)?r:g;
            max = (max > b)?max:b;
            int alpha = 255 - max;
            //data[i]= (alpha << 24) | (b << 16) | (g << 8) | r ;
            data[i]=(alpha << 24);
        }

        // Avoid stupid warnings...
        *dummy  = (char)idummy;
        idummy  = (int)*dummy;
        idummy  = q;

        // Pixel Extraction
        fclose(fp);
        free(charImage);
        return true;
    }
}

}// END PPM LOADER =============================================================

#ifndef TEXTURES_H
#define TEXTURES_H

#define STB_IMAGE_IMPLEMENTATION
#include"Textures/stb_image.h"

unsigned int create_1_texture(const char* imagePath, bool transparency){
	stbi_set_flip_vertically_on_load(true);

	int channels = transparency ? GL_RGBA : GL_RGB; 

	unsigned int ID;
	glGenTextures(1, &ID);
    glBindTexture(GL_TEXTURE_2D, ID); //Bind it

    //Opciones de textura

    //Las coordenadas a usar y como se manejan malas coordenadas
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    //Como se escala
    //Mipmap changes sizes depending of distance to object
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    //Leemos la imagen
    int ImgWidth, ImgHeight, nrChannels;
    unsigned char *data = stbi_load(imagePath, &ImgWidth, &ImgHeight, &nrChannels, 0); 

    if(data){
        //La llenamos
        //Where, which mipmap level, channels to store texture, size, , 0, channels of original, data type, image
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, ImgWidth, ImgHeight, 0, channels, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

    }else{
        std::cout << "Error al cargar textura" << std::endl;
    }
    stbi_image_free(data);

	return ID;
}


#endif
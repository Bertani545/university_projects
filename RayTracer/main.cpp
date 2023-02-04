#include<iostream>
#include<math.h>
#include"geometria.hpp"
#include<cstdlib>

#include <chrono>
using namespace std::chrono;

/*No habra reflexion ni refraccion
Todas las luces tienen por 1 sus componentes especulares, difusos y ambientales
*/

/*Formato del input:

                                        ImageHeight ImageWidth



eje y grados de rotacion de la camara   x y z theta

traslacion de la camara                 x y z

Numero de luces                         n
n posiciones de luces                   x y z
.
.
.

Iluminacion global                      k

*/

int main(){
    srand(time(NULL));
    auto start = high_resolution_clock::now(); //Para medir el tiempo
    
    /*--------------------------------------------------- Pantalla -----------------------------------------*/
    /*Valores defaults
    https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-generating-camera-rays/generating-camera-rays*/

    vec camera(0.0f, 0.0f, 0.0f); //Posicion default de la Camara
    
    double alpha = 90.0; //Campo de vision
    alpha = alpha*MY_PI/180.0; //A radianes

    //Distancia de la camara a la pantalla, Default
    vec CtVC(0.0f, 0.0f, -1.0f);

    //Resolucion de la imagen 
    int res1, res2;
    std::cin >> res1 >> res2;
    int resolution[2] = {res1, res2};
    
    //Para hacerlos cuadrados
    double ratio = resolution[0]/(float)resolution[1];
    
    //Constante para el supersampling
    unsigned int SuperS = 0;
    std::cin >> SuperS;
    if(SuperS < 1) SuperS = 1;
    
    //Creamos la matriz donde se guardara la imagen
    color** matImg = new color*[resolution[0]];
    for(int i = 0; i < resolution[0]; i++){
        matImg[i] = new color[resolution[1]];     
    }

    //Reescalamos la resolucion para el sampling que sera de SuperS*SuperS
    resolution[0] = resolution[0] * SuperS;
    resolution[1] = resolution[1] * SuperS;

    //Obtenemos el size de un pixel
    double pixelX = 2.0/resolution[0] * ratio * tan(alpha/2.0);
    double pixelY = 2.0/resolution[1] * tan(alpha/2.0);


    //Creamos una matriz de centros de los pixeles
    vec** centros = new vec*[resolution[0]];
    for(int i = 0; i < resolution[0]; i++){
        centros[i] = new vec[resolution[1]];
    }

    //Calculamos los centros
    /*Encontramos el primer valor del centro del pixel en la esquina superior izquierda*/
    vec firstV(-1.0 * ratio * tan(alpha/2.0) + pixelX/2.0, 1.0 * tan(alpha/2.0) - pixelY/2.0, -1.0f);

    //Sumamos el size del pixel para cada centro nuevo
    for(int i = 0; i < resolution[0]; i++){
        for(int j = 0; j < resolution[1]; j++){
            centros[i][j] = firstV  + vec(i*pixelX, -j*pixelY, 0.0f);
        }
    }
    
    /*-------------------------------- Movemos la camara y pantalla -------------------------------------------*/

    //Rotacion
    double ejeX, ejeY, ejeZ, grados;
    std::cin >> ejeX >> ejeY >> ejeZ >> grados;
    camera = rotaVec(camera, vec(ejeX,ejeY,ejeZ), grados);
    for(int i = 0; i < resolution[0]; i++){
        for(int j = 0; j < resolution[1]; j++){
            centros[i][j] = rotaVec(centros[i][j], vec(ejeX,ejeY,ejeZ), grados);
        }
    }

    //Traslacion
    double dx, dy, dz;
    std::cin >> dx >> dy >> dz;
    vec traslada(dx, dy, dz);
    
    //Traslacion
    camera = camera + traslada;
    for(int i = 0; i < resolution[0]; i++){
        for(int j = 0; j < resolution[1]; j++){
            centros[i][j] = centros[i][j] + traslada;
        }
    }
    //Creamos un plano de la pantalla para verificar que estemos del lado correcto
    plano pantalla(triangulo(centros[0][0], centros[1][0], centros[0][1])); //CCW

    /*--------------------------------- fin camera y pantalla ----------------------------------------------------*/

    /*-------------------------------------- Luces ----------------------------------------------------*/
    int n;
    std::cin >> n;
    std::vector<vec> luces(n);
    for(int i = 0; i < n; i++){
        double x,y,z;
        std::cin >> x >> y >> z;
        luces[i] = vec(x,y,z);
    }

    //La constante de iluminacion global. Truncada a 1 si se excede
    double globalIlumination;
    std::cin >> globalIlumination;
    if(globalIlumination > 1) globalIlumination = 1.0;

    /*---------------------------------------------------- Fin -----------------------------------------*/


    /*----------------------------------------------- Escena-----------------------------------------*/
    //Definimos todos los objetos de escena
    std::vector<mallaTriangular> mallasTriangulares;
    std::vector<esfera> esferas;
    std::vector<cubo> cubos;

    //mallas
    mallasTriangulares.push_back(mallaTriangular("link.obj", "texturaLink.jpg"));

    //Esferas
    esferas.push_back(esfera(vec(-30, 130, 10), 10, color(255, 255, 255), 0.5, 0.5, 32));

    //Cubos. P1.z > P2.z
    cubos.push_back(cubo(vec(30,30,10), vec(50,70, -10), color(134, 251, 255), 45,  0.8, 0.8, 32));


    //Calculamos el color de los pixeles

    //Para cada uno de los pixeles de la imagen
    int progress = 10;
    for(int i = 0; i < resolution[0]/SuperS; i++){
        for(int j = 0; j < resolution[1]/SuperS; j++){

            color pixelColor(0,0,0); //Color inicial
            bool found = false; //Avisa de intersecciones

            //Para cada uno de los 9 pixeles de la pantalla correspondientes
            //al pixel de la imagen
            for(int is = 0; is < SuperS; is++){
                for(int js = 0; js < SuperS; js++){

                    //Encontramos el punto mas cercano a la pantalla intersecando con todos los objetos
                    punto p(vec(MAX/2.0, MAX/2.0, MAX/2.0), vec(0,0,0), color(0,0,0));
                    punto curr;
                    //Esferas
                    for(int recorre = 0; recorre < esferas.size(); recorre++){
                        curr = closestInterRayYEsfera(ray(camera, centros[SuperS*i+is][SuperS*j+js]-camera), esferas[recorre]);
                        if((curr.position != vec(MAX, MAX, MAX) && distance(camera, curr.position) > distance(camera, centros[SuperS*i+is][SuperS*j+js])) && 
                            (distance(camera, curr.position) < distance(camera, p.position) && dotProd(pantalla.eta, curr.position - centros[SuperS*i+is][SuperS*j+js]) > 0)){
                            p = curr;
                            found = true;
                        }
                    }
                    


                    //Cubos
                    for(int recorre = 0; recorre < cubos.size(); recorre++){
                        curr = closestInterRayCube(ray(camera, centros[SuperS*i+is][SuperS*j+js]-camera), cubos[recorre]);
                        if((curr.position != vec(MAX, MAX, MAX) && distance(camera, curr.position) > distance(camera, centros[SuperS*i+is][SuperS*j+js])) && 
                            (distance(camera, curr.position) < distance(camera, p.position) && dotProd(pantalla.eta, curr.position - centros[SuperS*i+is][SuperS*j+js]) > 0)){
                            p = curr;
                            found = true;
                        }
                    }

                    

                    //Mallas triangulares
                    for(int recorre = 0; recorre < mallasTriangulares.size(); recorre++){
                        curr = closestInterRayTriangulos(ray(camera, centros[SuperS*i+is][SuperS*j+js] - camera), &mallasTriangulares[recorre]);
                        if((curr.position != vec(MAX, MAX, MAX) && distance(camera, curr.position) > distance(camera, centros[SuperS*i+is][SuperS*j+js])) && 
                            (distance(camera, curr.position) < distance(camera, p.position) && dotProd(pantalla.eta, curr.position - centros[SuperS*i+is][SuperS*j+js]) > 0)){
                            p = curr;
                            found = true;
                        }
                    }
                    
                    //Calculamos la iluminacion de phong
                    double diff = 0;
                    double spec = 0;

                    //si interseco o si se ve ocupamos los componentes especular y difuso
                    if(p.normal != vec(0,0,0)){
                        
                        bool foundLight = true;
                        //Verificamos que no este en sombra para todas las luces
                        for(int l = 0; l < luces.size(); l++){
                            foundLight = true;

                            //Intersecamos para todos los puntos        
                            vec lightDir = luces[l]-p.position;
                            ray rayoLuz(p.position, lightDir);

                            //Esferas
                            for(int recorre = 0; recorre < esferas.size(); recorre++){
                                curr = closestInterRayYEsfera(rayoLuz, esferas[recorre]);
                                if((curr.position != vec(MAX, MAX, MAX) && distance(curr.position, p.position) > TOL)&&
                                    distance(p.position, curr.position) < distance(p.position, luces[l]) && curr.position.x/rayoLuz.dir.x > 0){
                                    foundLight = false;
                                }
                            }
                            

                            //Cubos
                            for(int recorre = 0; recorre < cubos.size(); recorre++){
                                curr = closestInterRayCube(rayoLuz, cubos[recorre]);
                                if((curr.position != vec(MAX, MAX, MAX) && distance(curr.position, p.position) > TOL)&&
                                    distance(p.position, curr.position) < distance(p.position, luces[l]) && curr.position.x/rayoLuz.dir.x > 0){
                                    foundLight = false;
                                }
                            }
                            

                            //Mallas triangulares
                            for(int recorre = 0; recorre < mallasTriangulares.size(); recorre++){
                                if(checkShadowMalla(rayoLuz, &mallasTriangulares[recorre], luces[l])) foundLight = false;
                            }


                            //Si no hubo intersecciones
                            if(foundLight){
                                //Calculamos los componentes especular y difuso con la formula

                                /*https://learnopengl.com/Lighting/Basic-Lighting

                                https://en.wikipedia.org/wiki/Phong_reflection_model*/

                                p.normal.normalize();

                                //Luz difusa
                                lightDir.normalize();
                                diff += std::max(p.kd * dotProd(p.normal, lightDir), 0.0);

                                
                                //Luz especular
                                vec viewer = camera - centros[SuperS*i+is][SuperS*j+js];
                                viewer.normalize();
                                vec rebote = p.normal*2*std::max(dotProd(lightDir, p.normal), 0.0) - lightDir;
                                rebote.normalize();
                                spec += std::max(p.ks * pow(dotProd(viewer, rebote), p.alpha), 0.0);
                            }
                            //Si estaba en sombra pues no se tienen nada de esos componentes
                        }   

                        //Sumamos todos los componentes
                        pixelColor += (p.RGB*std::min((globalIlumination + diff + spec), 1.0));
                        /*
                            Notemos que las sombras no seran negras, si no del color
                            atenuado por la iluminacion global
                        */

                    }
                    /*Si se quiere un atardecer aqui tambien sumar
                    matImg[i][j]+= color(255, 255 - 255.0 * j/(float)resolution[1], 0);*/
                    
                }
            }
            /*Si no hay NINGUNA interseccion, colocamos el fondo de estrellas, pues lo hacemos
            aleatorio y por lo tanto no es constante*/
            if(!found){
                matImg[i][j] = color(0,0,0);
                if(rand()%100 == 0){
                    matImg[i][j] = color(255,255,255);
                }
            }else{
                //Al menos una interseccion hacemos el promedio
                matImg[i][j] = pixelColor/SuperS/SuperS;
            }
            
        }
        //Imprimimos lo que lleva de procesado el ray caster
        if( ((i+1)*resolution[0])/(resolution[0]*resolution[1]/(float)SuperS)*100 >= progress){
            std::cout << progress << "%" << std::endl;
            progress += 10;
        }
    }

    //Regresamos a las resoluciones iniciales
    resolution[0] = resolution[0]/SuperS;
    resolution[1] = resolution[1]/SuperS;
    
    //Creamos la imagen con la resolucion deseada usando Cimg
    CImg<unsigned char> result(resolution[0], resolution[1], 1, 3, 255);
    for(int i = 0; i < resolution[0]; i++){
        for(int j = 0; j < resolution[1]; j ++){
            unsigned char curCol[3];
            curCol[0] = matImg[i][j].R();
            curCol[1] = matImg[i][j].G();
            curCol[2] = matImg[i][j].B();
            result.draw_point(i,j, curCol);
        }
    }
    //Guardamos la imagen
    result.save("sample.png");
    
    //Imprimios cuanto se tardo
    auto stop = high_resolution_clock::now();

    auto durationChrono = duration_cast<seconds>(stop-start);
    int minutos = durationChrono.count()/60;
    int segundos = durationChrono.count() - 60*minutos;

    std::cout << "La imagen se completo' en " << minutos << " minutos y " <<
                    segundos << " segundos." << std::endl;

    //Liberamos la memoria dinamica
    delete[] matImg;
    delete[] centros;
    return 0;
}

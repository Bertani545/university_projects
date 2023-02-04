#ifndef GEOMETRIA_HPP
#define GEOMETRIA_HPP

#include<iostream>
#include<math.h>
#include<limits>
#include<vector>
#include<cstdlib>
#include<string>
#include <fstream>
/*---------------------------- Libreria para la imagen ------------------------*/
#include"CImg/CImg.h"

using namespace cimg_library;

#ifndef cimg_imagepath
#define cimg_imagepath "img/"
#endif

/*---------------------------------------------------- Fin -----------------------------------------*/

/*-------------------------------------------------- Definiciones -----------------------------------------*/

//Definimos algunas constantes y variables que usaremos en el codigo


# define MY_PI 3.14159265358979323846 //M_PI Solo esta definida en GNU

//Definir limites
double MAX = std::numeric_limits<double>::max();    
double MIN = std::numeric_limits<double>::min();
//Cabe resaltar que siempre que no haya interseccion, se rregresara un vector con MAX como entradas

//Para errores

double TOL = 10e-10; //Para tomar en cuenta errores numericos
bool eqDouble(double a, double b){
    return abs(a-b) < TOL;
}

/*---------------------------------------------------- Fin -----------------------------------------*/

/*-------------------------------------------- Vector en R3 --------------------------------------*/
class vec{
public:
    double x = 0;
    double y = 0;
    double z = 0;
    vec(){};
    vec(double X, double Y, double Z){
        this->x = X;
        this->y = Y;
        this->z = Z;
    };
    ~vec(){};
    
    //Definimos las operaciones que podemos hacer con vectores
    vec operator + (vec const &obj){
        vec res;
        res.x = this->x + obj.x;
        res.y = this->y + obj.y;
        res.z = this->z + obj.z;
        return res;
    }

    vec operator - (vec const &obj){
        vec res;
        res.x = this->x - obj.x;
        res.y = this->y - obj.y;
        res.z = this->z - obj.z;
        return res;
    }

    vec operator * (double const &d){
        vec res;
        res.x = this->x*d;
        res.y = this->y*d;
        res.z = this->z*d;
        return res;
    }

    vec operator / (double const &d){
        vec res;
        res.x = this->x/d;
        res.y = this->y/d;
        res.z = this->z/d;
        return res;
    }

    bool operator == (vec const &v){
        //Esta la usamos para verificaciones exactas
        if(this->x != v.x)
            return false;
        if(this->y != v.y)
            return false;
        if(this->z != v.z)
            return false; 
        return true;
    }

    bool operator != (vec const &v){
        //Para verificaciones resultado de operaciones
        vec r = *this - v;
        if(r.norm() < TOL) return false;
        return true;
    }


    //Calculo de la norma
    double norm(){
        return sqrt(x*x + y*y + z*z);
    }

    //Para normalizar el vector
    void normalize(){
        double norma = this->norm();
        this->x = x/norma;
        this->y = y/norma;
        this->z = z/norma; 
    }
};

/*--------------------------------------------- Operadores de vectores -----------------------------------*/
//Se obtienen por definicion

//Producto punto
double dotProd(vec a, vec b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

//La norma
double norma(vec v){
    return sqrt(dotProd(v,v));
}

//Producto cruz
vec prodCruz(vec u, vec v){
    return vec(u.y*v.z - u.z*v.y,
                u.z*v.x - u.x*v.z,
                u.x*v.y - u.y*v.x);
}

//Distancia entre dos vectores
double distance(vec a, vec b){
    return norma(a - b);
}

/*---------------------------------------------------- Fin -----------------------------------------*/

/*--------------------------------------------------- Rotaciones ---------------------------------*/

//Lo implementamos con cuaternios porque lo vimos en clase y me lo antojaron
class cuaternio{
public:
    double a = 0,b = 0, c = 0, d = 0;

    cuaternio(){};

    //El cuaternio que rota "theta" a traves del eje "eje"
    cuaternio(vec eje, double theta){
        eje.normalize();
        this->a = cos(theta/2.0);
        this->b = eje.x * sin(theta/2.0);
        this->c = eje.y * sin(theta/2.0);
        this->d = eje.z * sin(theta/2.0);
    };
    //Para la rotacion
    cuaternio(vec v){
        this->a = 0;
        this->b = v.x;
        this->c = v.y;
        this->d = v.z;
    }
    //Algun otro uso
    cuaternio(double A, double B, double C, double D){
        this->a = A;
        this->b = B;
        this->c = C;
        this->d = D;
    }

    ~cuaternio(){};
    
    //Definimos la multiplicacion de cuaternios
    cuaternio operator * (cuaternio const q){
        cuaternio res;
        res.a = this->a * q.a - this->b * q.b - this->c * q.c - this->d * q.d;
        res.b = this->a * q.b + this->b * q.a + this->c * q.d - this->d * q.c;
        res.c = this->a * q.c + this->c * q.a + this->d * q.b - this->b * q.d;
        res.d = this->a * q.d + this->d * q.a + this->b * q.c - this->c * q.b;
        return res;
    }

    //Para regresar la inversa del cuaternio unitario
    cuaternio inv(){
        return cuaternio(
            this->a,
            -(this->b),
            -(this->c),
            -(this->d)
            );
    };
};


/*Rotamos el punto ptn respecto al eje "eje" tantos grados como
"theta" nos indique*/
vec rotaVec(vec ptn, vec eje, double theta){
    eje.normalize();
    cuaternio q(eje, theta*MY_PI/180.0);
    cuaternio p(ptn);
    cuaternio rot = q*p*q.inv();
    vec res;
    res.x = rot.b;
    res.y = rot.c;
    res.z = rot.d;
    return res;
}

/*---------------------------------------------------- Fin -----------------------------------------*/

/*----------------------------------------------- Texturas y color ---------------------------------------*/

//vec2 exclusivamente usado para las texturas. Tiene los mismos operadores que vec
class vec2{
public:
    double u = 0;
    double v = 0;
    vec2(){};
    vec2(double U, double V){
        this->u = U;
        this->v = V;
    };

    vec2 operator + (vec2 const &obj){
        vec2 res;
        res.u = this->u + obj.u;
        res.v = this->v + obj.v;
        return res;
    }

    vec2 operator - (vec2 const &obj){
        vec2 res;
        res.u = this->u - obj.u;
        res.v = this->v - obj.v;
        return res;
    }

    vec2 operator * (double const &d){
        vec2 res;
        res.u = this->u*d;
        res.v = this->v*d;
        return res;
    }
};

//Definimos el color. 
/*Dado que Cimg usa un arreglo de unsigned char, nostros hacemos un vector de int
para poder sumar sin desbordarnos y que posteriormente se pueda castear al tipo deseado*/
class color{
    //Enteros para poder hacer promedios
public:
    int RGB[3] = {0,0,0};
    color(){};
    color(unsigned char r, unsigned char g, unsigned char b){
        this->RGB[0] = r;
        this->RGB[1] = g;
        this->RGB[2] = b;
    };
    ~color(){};

    //Operaciones usuales con colores
    color operator += (color const &c){
        this->RGB[0] += c.RGB[0];
        this->RGB[1] += c.RGB[1];
        this->RGB[2] += c.RGB[2];
        return *this;
    }
    color operator *(double const d){
        color res;
        res.RGB[0] = this->RGB[0]*d;
        res.RGB[1] = this->RGB[1]*d;
        res.RGB[2] = this->RGB[2]*d;
        return res;
    }
    color operator *(vec const v){
        color res;
        res.RGB[0] = this->RGB[0] * v.x;
        res.RGB[1] = this->RGB[1] * v.y;
        res.RGB[2] = this->RGB[2] * v.z;
        return res;
    }
    color operator +(color const c){
        color res;
        res.RGB[0] = this->RGB[0] + c.RGB[0];
        res.RGB[1] = this->RGB[1] + c.RGB[1];
        res.RGB[2] = this->RGB[2] + c.RGB[2];
        return res;
    }
    color operator /(double const d){
        color res;
        res.RGB[0] = this->RGB[0]/d;
        res.RGB[1] = this->RGB[1]/d;
        res.RGB[2] = this->RGB[2]/d;
        return res;
    }

    //Por si solo queremos modificar un valor
    unsigned char R(){return this->RGB[0];}
    unsigned char G(){return this->RGB[1];}
    unsigned char B(){return this->RGB[2];}
    void R(unsigned char r){this->RGB[0] = r;}
    void G(unsigned char g){this->RGB[1] = g;}
    void B(unsigned char b){this->RGB[2] = b;}

};

/*---------------------------------------------------- Fin -----------------------------------------*/

/*--------------------------------------------------- Puntos -----------------------------------------*/

/*La clase punto contiene toda la informacion necesaria para renderizar:
    - La coordenada del punto
    - La normal al punto
    - El color en ese punto
    - Las constantes utilizadas para iluminar una superficie con phong

No se hacen operaciones con esta clase pues no hace sentido teniendo vec
*/
class punto{
public:
    vec position;
    vec normal;
    color RGB;
    double ks = 0, kd = 0, alpha = 0;


    punto(){this->RGB = color(0,0,0);};
    punto(vec p, vec n, color c){
        this->position = p;
        this->normal = n;
        this->RGB = c;
    };
    punto(vec p, vec n, color c, double KS, double KD, double A){
        if(KS > 1) KS = 1;
        if(KD > 1) KD = 1;
        this->position = p;
        this->normal = n;
        this->RGB = c;
        this->ks = KS;
        this->kd = KD;
        this->alpha = A;
    };
    ~punto(){};
    
};

/*---------------------------------------------------- Fin -----------------------------------------*/


/*---------------------------------------------------- Rayos -----------------------------------------*/

/*
    Los consideraremos de la forma C+lV para l > 0, donde C y V son vectores
*/
class ray{
public:
    //Centro y vec direccion
    vec centro;
    vec dir; //normalizada

    ray(vec C, vec V){
        this->centro = C;
        this->dir = V;
        this->dir.normalize();
    };
    ~ray(){};
};



/*---------------------------------------------------- Fin -----------------------------------------*/


/*------------------------------------------------- Triangulos -----------------------------------------*/

/*Pude haberlo hecho con puntos pero dicha clase la implemente muuucho despues que triangulo asi que
    ya se quedo asi para evitar problemas y tener que reescribir muchas funciones*/

/*Para la malla triangular. Solo guarda informacion que otras funciones utilizaran*/

class triangulo{
public:
    vec p1, p2, p3; //Vertices
    vec n1, n2, n3; //Normales
    vec2 t1, t2, t3;   //Coordenada textura
    double ks = 0, kd = 0, alpha = 0;

    triangulo(vec a, vec b, vec c){
        this->p1 = a;
        this->p2 = b;
        this->p3 = c;

        this->n1 = prodCruz(this->p2-this->p1, this->p3 - this->p1);
        this->n1.normalize();
        this->n2 = this->n1;
        this->n3 = this->n1;

        this->t1 = vec2(0.0f, 0.0f);
        this->t2 = vec2(0.0f, 0.0f);
        this->t3 = vec2(0.0f, 0.0f);
    };

    triangulo(vec P1, vec P2, vec P3, vec N1, vec N2, vec N3, 
                    vec2 T1,  vec2 T2,  vec2 T3){
        this->p1 = P1;
        this->p2 = P2;
        this->p3 = P3;
        this->n1 = N1;
        this->n2 = N2;
        this->n3 = N3;
        this->t1 = T1;
        this->t2 = T2;
        this->t3 = T3;

    };
    triangulo(vec P1, vec P2, vec P3, vec N1, vec N2, vec N3, 
                    vec2 T1,  vec2 T2,  vec2 T3, double KS, double KD, double A){
        this->p1 = P1;
        this->p2 = P2;
        this->p3 = P3;
        this->n1 = N1;
        this->n2 = N2;
        this->n3 = N3;
        this->t1 = T1;
        this->t2 = T2;
        this->t3 = T3;

        if(KS > 1) KS = 1;
        if(KD > 1) KD = 1;
        this->ks = KS;
        this->kd = KD;
        this->alpha = A;
    };
    triangulo(){
        this->p1 = vec(0.0f, 0.0f, 0.0f);
        this->p2 = vec(0.0f, 0.0f, 0.0f);
        this->p3 = vec(0.0f, 0.0f, 0.0f);
    };
    ~triangulo(){};
};




//Funcion que regresa el punto mas cercano a centro. Se utiliza solo 1 vez para la esfera

//Regresa un vector con MAX si la lista esta vacia
vec closestToPoint(vec center, std::vector<vec> points){
    double min = std::numeric_limits<double>::max();
    int cur = -1;
    for(int i = 0; i < points.size(); i++){
        if(norma(points[i]) < min)
            min = norma(points[i]);
            cur = i;
    }
    if(cur != -1)
        return points[cur];
    //std::cout << "Vector vacio" << std::endl;
    return vec(MAX, MAX, MAX);
}   



//Definimoos los planos para poder calcular la interseccion de un rayo con un triangulo
/*
Recordemos que todo triangulo define SOLO un plano, de ahi que haya un constructor con 
parametro triangulo
*/
class plano{
public:
    //El centro y los 2 vectores generadores ortonormales
    /*Formula:
        (p-p_0).eta*/
    vec eta;
    vec p0;
    plano(vec P, vec Eta){
        this->p0 = P;
        this->eta = Eta;
        
    };
    plano(triangulo tr){
        //Construimos el plano definido por los 3 puntos
        this->p0 = tr.p1;
        vec v1 = tr.p2 - tr.p1;
        vec v2 = tr.p3 - tr.p1;
        this->eta = prodCruz(v1, v2);
        this->eta = this->eta/norma(this->eta);
    };
    plano(){};
    ~plano(){};
};

//Funcion para calcular la interseccion de un plano y un rayo
/*https://www.wikiwand.com/en/Line–plane_intersection*/
vec interPlanoyRayo(plano PL, ray RAYO){
    //Paralelos
    if(abs(dotProd(PL.eta, RAYO.dir)) < TOL)
        return vec(MAX, MAX, MAX);

    //A que distancia del centro del rayo.
    //Si es menor a 0 l orechazamos pues esta detras
    double d = dotProd(PL.p0 - RAYO.centro, PL.eta)/dotProd(RAYO.dir, PL.eta);
    if(d > -TOL)
        return RAYO.centro + (RAYO.dir*d);

    return vec(MAX, MAX, MAX);
}


//Una vez encontramos la interseccion con el plano, debemos ver si cae dentro del triangulo
//Para esto usamos coordenadas baricentricas
    /*https://math.stackexchange.com/questions/4322/check-whether-a-point-is-within-a-3d-triangle*/
bool isInsideTriangle(vec v, triangulo t){
    if(v == vec(MAX, MAX, MAX))
        return false;

    /*Si un punto cae dentro del triangulo, las coordenadas suman 1*/
    double area = norma(prodCruz(t.p2 - t.p1, t.p3 - t.p1))/2;
    double alpha = norma(prodCruz(t.p2 - v, t.p3 - v))/(2*area);
    double beta = norma(prodCruz(t.p3 - v, t.p1 - v))/(2*area);
    double gamma = norma(prodCruz(t.p2 - v, t.p1 - v))/(2*area);

    if(abs(alpha + beta + gamma -1) >= TOL)//Suman > 1?
        return false;

    //Suman 1 y por lo tanto esta dentro
    return true;
}





/*
funcion para la interseccion de un rayo y un triangulo. Si esta dentro
regresamos el punto, si no, uno con MAX en las entradas
 */
vec interRayAndTriangle(ray RAYO, triangulo tri){
    vec p = interPlanoyRayo(plano(tri), RAYO);
    if(isInsideTriangle(p, tri))
        return p;
    return vec(MAX, MAX, MAX);
}


/*
Funcion para calcular las coordenadas baricentricas de un punto v para el triangulo t
Esta lo usaremos para asignar las texturas a los triangulos de la malla
*/
vec coordBaricentricas(vec v, triangulo t){
    double area = norma(prodCruz(t.p2 - t.p1, t.p3 - t.p1))/2;
    double alpha = norma(prodCruz(t.p2 - v, t.p3 - v))/(2*area); //para p1
    double beta = norma(prodCruz(t.p3 - v, t.p1 - v))/(2*area); //Para p2
    double gamma = norma(prodCruz(t.p2 - v, t.p1 - v))/(2*area); //para p3
    return vec(alpha, beta, gamma);
}

/*---------------------------------------------------- Fin -----------------------------------------*/

/*-------------------------------------------------- Esferas -----------------------------------------*/

/*Clase para definir la esfera con:
    -Centro
    -Radio
    -Color
    -Conostantes para phong

Dos constructores pues tambien las usaremos como hitboxes
*/
class esfera{
public:
    vec centro;
    double r = 0;
    color RGB;
    double ks = 0, kd = 0, alpha = 0;

    esfera(){};
    esfera(vec C, double R){
        this->centro = C;
        this->r = R;
    };
    esfera(vec C, double R, color col, double KS, double KD, double A){
        if(KS > 1) KS = 1;
        if(KD > 1) KD = 1;
        this->centro = C;
        this->r = R;
        this->RGB = col;
        this->ks = KS;
        this->kd = KD;
        this->alpha = A;
    };
    ~esfera(){};
};


//Funcion de la interseccion de una esfera y un rayo
/*https://en.wikipedia.org/wiki/Line–sphere_intersection*/

/*Recordemos que la normal en un punto p de una esfera es el vector que
va del centro de la esfera a dicho punto*/
punto closestInterRayYEsfera(ray rayo, esfera esf){

    //Discriminante
    double delta = dotProd(rayo.dir, rayo.centro - esf.centro)*dotProd(rayo.dir, rayo.centro - esf.centro)
                    -(dotProd(rayo.centro - esf.centro, rayo.centro - esf.centro) - esf.r*esf.r);

    //Solo guardamos aquellos puntos que esten en direccion del rayo, de ahi las comparaciones de d > 0
    if(eqDouble(delta, 0.0)){//Una sola solucion
        double d = -dotProd(rayo.dir, rayo.centro - esf.centro);
        if(d > -TOL){
            return punto(rayo.centro + rayo.dir*d, rayo.centro + rayo.dir*d - esf.centro , esf.RGB, esf.ks, esf.kd, esf.alpha);
        }
        
    }
    if(delta > 0){//Dos soluciones
        double d1 = -dotProd(rayo.dir, rayo.centro - esf.centro) + sqrt(delta);
        double d2 = -dotProd(rayo.dir, rayo.centro - esf.centro) - sqrt(delta);
        std::vector<vec> posibles;

        //Verificamos cuales estan del lado del rayo y regresamos el menor
        if(d1 > -TOL)
            posibles.push_back(rayo.centro + rayo.dir*d1);
        if(d2 > -TOL)
            posibles.push_back(rayo.centro + rayo.dir*d2);

        vec p = closestToPoint(rayo.centro, posibles);

        return punto(p, p - esf.centro, esf.RGB, esf.ks, esf.kd, esf.alpha);
    }
    
    //No hay solucion, no hay interseccion
    return punto(vec(MAX, MAX, MAX), vec(0,0,0), color(0,0,0), 0,0,0);

}

//Verifica la interseccion de un rayo y una esfera pero no calcula puntos, regresa un booleano
//Esta funcion es para las hitoboxes
bool rayInterEsfera(ray rayo, esfera esf){
    double delta = dotProd(rayo.dir, rayo.centro - esf.centro)*dotProd(rayo.dir, rayo.centro - esf.centro)
                    -(dotProd(rayo.centro - esf.centro, rayo.centro - esf.centro) - esf.r*esf.r);

    if(delta > TOL && ((-dotProd(rayo.dir, rayo.centro - esf.centro) + sqrt(delta)) > 0 || 
        (-dotProd(rayo.dir, rayo.centro - esf.centro) - sqrt(delta)) > 0) ) return 1;
    return 0;
}   


/*---------------------------------------------------- Fin -----------------------------------------*/

/*--------------------------------------------------- Cubos -----------------------------------------*/

/*
Podemos definir a un cubo con dos puntos y un angulo.
    Los puntos corresponden a la diagonal y el angulo a cuanto giramos los demas
puntos respescto a dicha diagonal.

Para nuestra implementación, consideramos que en el diagrama siguiente, nos dieron P1 y P7
Por lo que P1.z <  P7.z. Si esto no ocurre los voltearemos para evitar problemas

La clase tambien tiene las constantes para la iluminacion de phong
*/

/*
https://inmensia.com/articulos/raytracing/planotrianguloycubo.html*/
/*  
     P8/-----p7
    P4-----/P3
      |   |
    P1-----/P2
    p5      p6
*/
class cubo{
    /*Siempre el P1 con un eje z menor que P2*/
public:
    vec p1, p2, p3, p4, p5, p6, p7, p8;
    double ks = 0, kd = 0, alpha = 0;
    esfera  hitbox;
    /*Normales
    n1 para p1 p2 p3 p4
    n2 para p5 p6 p7 p8
    n3 para p1 p5 p8 p4
    n4 para p2 p6 p7 p3
    n5 para p1 p2 p6 p5
    n6 para p4 p3 p7 p8
    */
    vec n1 = vec(0, 0, 1), 
    n2 = vec(0,0,-1),
    n3 = vec(-1,0,0),
    n4 = vec(1,0,0),
    n5 = vec(0,-1,0),
    n6 = vec(0,1,0);

    color RGB;

    double l = 0;
    cubo(){};
    cubo(vec P1, vec P2, color col, double theta, double KS, double KD, double A){
        /*
        Para construirlo supondremos que comenzamos con un cubo en (0,0,0) a (l,l,l)
        y luego lo rotamos y trasladamos para que coincida con el definido por P1 y P2
        */

        //Primero nos aseguramos que P1.z < P2. z
        if(P1.z > P2.z){
            vec temp = P2;
            P2 = P1;
            P1 = temp;
        }
        //Para evitar errores
        if(KS > 1) KS = 1;
        if(KD > 1) KD = 1;

        //Guardamos las constantes y creamos la hitbox
        this->RGB = col;
        this->ks = KS;
        this->kd = KD;
        this->alpha = A;
        this->hitbox = esfera((P1+P2)/2.0, distance(P1,P2)/2.0);

        //Calculamos el size del lado
        this->l = distance(P1, P2)*sqrt(3.0)/3.0;

        //Encontramos el eje de rotacion para llevar (l,l,l) a P2
        vec dir1 = vec(l,l,-l), dir2 = P2-P1;
        vec eje = prodCruz(dir1, dir2);
        dir1.normalize(), dir2.normalize();
        double COS = dotProd(dir1, dir2);
        //Sabemos que es positivo
        COS = acos(COS)*180.0/MY_PI;

        //Creamos los demas puntos y normales rotandolos tambien respecto al eje P1P2 y luego
        //Trasladandolos a su posicion final
        this->p1 = P1;
        this->p7 = P2;
        vec ejeDiag = P2-P1;

        //Si son paralelos, rotarlos por un eje (0,0,0) no hace sentido por lo que los separamos
        if(norma(eje) > 0){
            this->p2 = rotaVec(rotaVec(vec(l,0,0), eje, COS), ejeDiag, theta) + P1;
            this->p3 = rotaVec(rotaVec(vec(l,l,0), eje, COS), ejeDiag, theta) + P1;
            this->p4 = rotaVec(rotaVec(vec(0,l,0), eje, COS), ejeDiag, theta) + P1;
            this->p5 = rotaVec(rotaVec(vec(0,0,-l), eje, COS), ejeDiag, theta) + P1;
            this->p6 = rotaVec(rotaVec(vec(l,0,-l), eje, COS), ejeDiag, theta) + P1;
            this->p8 = rotaVec(rotaVec(vec(0,l,-l), eje, COS), ejeDiag, theta) + P1;

            //normales;
            this->n1 = rotaVec(rotaVec(this->n1, eje, COS), ejeDiag, theta);
            this->n2 = rotaVec(rotaVec(this->n2, eje, COS), ejeDiag, theta);
            this->n3 = rotaVec(rotaVec(this->n3, eje, COS), ejeDiag, theta);
            this->n4 = rotaVec(rotaVec(this->n4, eje, COS), ejeDiag, theta);
            this->n5 = rotaVec(rotaVec(this->n5, eje, COS), ejeDiag, theta);
            this->n6 = rotaVec(rotaVec(this->n6, eje, COS), ejeDiag, theta);
        }else{
            //Son paralelos
            this->p2 = rotaVec(vec(l,0,0), ejeDiag, theta) + P1;
            this->p3 = rotaVec(vec(l,l,0), ejeDiag, theta) + P1;
            this->p4 = rotaVec(vec(0,l,0), ejeDiag, theta) + P1;
            this->p5 = rotaVec(vec(0,0,-l), ejeDiag, theta) + P1;
            this->p6 = rotaVec(vec(l,0,-l), ejeDiag, theta) + P1;
            this->p8 = rotaVec(vec(0,l,-l), ejeDiag, theta) + P1;

            //las normales solo cambian por la rotacion del eje
            this->n1 = rotaVec(this->n1, ejeDiag, theta);
            this->n2 = rotaVec(this->n2, ejeDiag, theta);
            this->n3 = rotaVec(this->n3, ejeDiag, theta);
            this->n4 = rotaVec(this->n4, ejeDiag, theta);
            this->n5 = rotaVec(this->n5, ejeDiag, theta);
            this->n6 = rotaVec(this->n6, ejeDiag, theta);
        }
    };
    ~cubo(){};
};

//Funcion para encontrar la interseccion de un rayo y un cubo
/*
    Simplemente vemos si choca con algun plano definido por una cara y si esta dentro
    Obtenemos el minimo de todas las intersecciones

    Si no se encontro el minimo, se regresara un vec con entradas MAX
*/
punto closestInterRayCube(ray r, cubo c){
    //Verificamos las 6 caras
    /*  
     P8/-----p7
    P4-----/P3
      |   |
    P1-----/P2
    p5      p6
*/
        /*Normales
    n1 para p1 p2 p3 p4
    n2 para p5 p6 p7 p8
    n3 para p1 p5 p8 p4
    n4 para p2 p6 p7 p3
    n5 para p1 p2 p6 p5
    n6 para p4 p3 p7 p8
    */
    if(!rayInterEsfera(r, c.hitbox)) return punto(vec(MAX, MAX, MAX), vec(0,0,0), color(0,0,0), 0, 0, 0);
    vec p;
    punto min(vec(MAX, MAX, MAX), vec(0,0,0), color(0,0,0), 0, 0, 0);
    //Perdoname por todo tiempo de ejecucion T__T

    //Cara 1
    p = interPlanoyRayo(plano(c.p1, c.n1), r);
    if(isInsideTriangle(p, triangulo(c.p1, c.p2, c.p3)) || isInsideTriangle(p, triangulo(c.p1, c.p4, c.p3))){
        if(distance(min.position, r.centro) > distance(p, r.centro)) min = punto(p, c.n1, c.RGB, c.ks, c.kd, c.alpha);  
    }
    //cara 2
    p = interPlanoyRayo(plano(c.p7, c.n2), r);
    if(isInsideTriangle(p, triangulo(c.p5, c.p6, c.p7)) || isInsideTriangle(p, triangulo(c.p5, c.p7, c.p8))){
        if(distance(min.position, r.centro) > distance(p, r.centro)) min = punto(p, c.n2, c.RGB, c.ks, c.kd, c.alpha);  
    }
    //Cara 3
    p = interPlanoyRayo(plano(c.p1, c.n3), r);
    if(isInsideTriangle(p, triangulo(c.p1, c.p5, c.p8)) || isInsideTriangle(p, triangulo(c.p1, c.p8, c.p4))){
        if(distance(min.position, r.centro) > distance(p, r.centro)) min = punto(p, c.n3, c.RGB, c.ks, c.kd, c.alpha);  
    }
    //Cara 4
    p = interPlanoyRayo(plano(c.p7, c.n4), r);
    if(isInsideTriangle(p, triangulo(c.p2, c.p6, c.p7)) || isInsideTriangle(p, triangulo(c.p2, c.p7, c.p3))){
        if(distance(min.position, r.centro) > distance(p, r.centro)) min = punto(p, c.n4, c.RGB, c.ks, c.kd, c.alpha);  
    }
    //Cara 5
    p = interPlanoyRayo(plano(c.p1, c.n5), r);
    if(isInsideTriangle(p, triangulo(c.p1, c.p2, c.p6)) || isInsideTriangle(p, triangulo(c.p1, c.p6, c.p5))){
        if(distance(min.position, r.centro) > distance(p, r.centro)) min = punto(p, c.n5, c.RGB, c.ks, c.kd, c.alpha);  
    }
    //Cara 6
    p = interPlanoyRayo(plano(c.p7, c.n6), r);
    if(isInsideTriangle(p, triangulo(c.p4, c.p3, c.p7)) || isInsideTriangle(p, triangulo(c.p4, c.p7, c.p8))){
        if(distance(min.position, r.centro) > distance(p, r.centro)) min = punto(p, c.n6, c.RGB, c.ks, c.kd, c.alpha);  
    }

    return min;
}

/*---------------------------------------------------- Fin -----------------------------------------*/


/*------------------------------------------ Mallas triangulares ------------------------------------*/

//Basicamente leer un archivo obj modificado que permite renderizar modelos con una textura
class mallaTriangular{
public:
    esfera hitbox;
    std::vector<triangulo> malla;
    
    CImg<unsigned char> textura;


    mallaTriangular(){};
    mallaTriangular(std::string objModPath, std::string texturePath){
        //Leemos el obj modificado
        std::ifstream obj;
        obj.open(objModPath);

        int n;
        double KS, KD, A;
        obj >> KS >> KD >> A;
        if(KS > 1) KS = 1;
        if(KD > 1) KD = 1;
        //PAra el hitbox
        double minX = MAX, minY = MAX, minZ = MAX;
        double maxX = MIN, maxY = MIN, maxZ = MIN;

        //Leemos vertices
        obj >> n;
        std::vector<vec> vertex(n);
        for(int i = 0; i < n; i++){
            double d1, d2, d3;
            obj >> d1 >> d2 >> d3;

            if(d1 < minX) minX = d1;
            if(d1 > maxX) maxX = d1;
            if(d2 < minY) minY = d2;
            if(d2 > maxY) maxY = d2;
            if(d3 < minZ) minZ = d3;
            if(d3 > maxZ) maxZ = d3;


            vertex[i] = vec(d1, d2, d3);
        }

        //Creamos la esfera que contiene al objeto
        /*la esfera que contiene al prisma definido por los maximos y minimos contiene al modelo*/
        this->hitbox = esfera(vec((maxX+minX)/2.0, (maxY+minY)/2.0, (maxZ+minZ)/2.0), 
                        distance(vec((maxX+minX)/2.0, (maxY+minY)/2.0, (maxZ+minZ)/2.0), vec(minX, minY, minZ)));

        //Coordenadas de textura
        obj >> n;
        std::vector<vec2> texture(n);
        for(int i = 0; i < n; i++){
            double t1, t2;
            obj >> t1 >> t2;
            texture[i] = vec2(t1,t2);
        }

        //Normales
        obj >> n;
        std::vector<vec> normals(n);
        for(int i = 0; i < n; i++){
            double d1, d2, d3;
            obj >> d1 >> d2 >> d3;
            normals[i] = vec(d1, d2 , d3);
        }

        //Creamos los triangulos
        obj >> n;
        this->malla.resize(n);
        for(int i = 0; i < n; i++){
            //Leemos los 3 vertices
            int v1,v2,v3,t1,t2,t3,n1,n2,n3;
            obj >> v1 >> t1 >> n1;
            obj >> v2 >> t2 >> n2;
            obj >> v3 >> t3 >> n3;

            malla[i] = triangulo(vertex[v1-1],vertex[v2-1], vertex[v3-1],
                normals[n1-1], normals[n2-1], normals[n3-1], texture[t1-1], texture[t2-1], texture[t3-1],
                KS, KD, A);
        }


        obj.close();
        //Guardamos la textura pues deberemos usar interpolacion 
        CImg<unsigned char> temp(texturePath.c_str());
        this->textura = temp;
    };
    ~mallaTriangular(){};
    
};

/*
    Funcion para verificar si un punto es sombreado por lo malla

    Iteramos sobre los triangulos y si hay una interseccion con la
    malla mas cercana a la luz que el punto, entonces estamos en sombra
*/

bool checkShadowMalla(ray r, mallaTriangular* m, vec L){
    for(int k = 0; k < (*m).malla.size(); k++){
        vec nuevo = interRayAndTriangle(r, (*m).malla[k]);
        if((nuevo != r.centro && distance(L, nuevo) < distance(L, r.centro))&& nuevo.x/r.dir.x > TOL)
            return true;
    }
    return false;
}

/*
    Funcion para regresar el punto de interseccion mas cercano entre un rayo y la malla
*/

punto closestInterRayTriangulos(ray r, mallaTriangular* m){
    //Verificamos si toca la hitbox
    if(!rayInterEsfera(r, (*m).hitbox)) return punto( vec(MAX, MAX, MAX), vec(0,0,0), color(0,0,0));

    vec p(MAX,MAX,MAX);
    //Iteramos sobre todos los triangulos para encontrar el minimo

    int min = -1; //Necesario para asignar textura
    for(int k = 0; k < (*m).malla.size(); k++){
        vec nuevo = interRayAndTriangle(r, (*m).malla[k]);
        if(nuevo != vec(MAX, MAX, MAX) && distance(r.centro, nuevo) < distance(r.centro, p)){
                p = nuevo;
                min = k;
        }
    }
    if(min != -1){//Existe interseccion
        //Coordenadas baricentricas
        vec bari = coordBaricentricas(p, (*m).malla[min]);

        //Su posicion en la textura
        vec2 pos = (*m).malla[min].t1 * bari.x + (*m).malla[min].t2 * bari.y + (*m).malla[min].t3 * bari.z;

        //Obtenemos el color usando la posicion anterior
        if(pos.u < 0) pos.u = 0;
        if(pos.v < 0) pos.v = 0;
        color col((*m).textura((int)((*m).textura.width() * pos.u), (int)((*m).textura.height() * (1.0 - pos.v)) - 1, 0, 0),
                (*m).textura((int)((*m).textura.width() * pos.u), (int)((*m).textura.height() * (1.0 - pos.v)) - 1, 0, 1),
                (*m).textura((int)((*m).textura.width() * pos.u), (int)((*m).textura.height() * (1.0 - pos.v)) - 1, 0, 2));

        //obtenemos normales interpolando con las coordenadas baricentricas
        vec n = (*m).malla[min].n1 * bari.x + (*m).malla[min].n2 * bari.y + (*m).malla[min].n3 * bari.z;

        return punto(p, n, col, (*m).malla[min].ks, (*m).malla[min].kd, (*m).malla[min].alpha);
    }
    //No hubo interseccion
    return punto( vec(MAX, MAX, MAX), vec(0,0,0), color(0,0,0));

}

/*---------------------------------------------------- Fin -----------------------------------------*/

#endif

                    
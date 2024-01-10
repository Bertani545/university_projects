#ifndef RRT_H
#define RRT_H

#include <vector>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <eigen3/Eigen/Dense>
#include <random>
#include "collisions.hpp"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Quaterniond;

//Podemos movernos hacia adelante (que es negativo)
//Podemos girar en todas direcciones
std::vector<Vector3d> UF = {{0,0, -0.5}, {0,0, 0.5}, {0,0,0}};  // 2 x 3
std::vector<Vector3d> UT = {{0.01, 0, 0}, {-0.01, 0, 0},
                            {0, 0.01, 0}, {0, -0.01, 0},
                            {0, 0, 0.01}, {0, 0, -0.01},
                            {0,0,0}};  // 6 x 3*/
double dt = 0.25;


struct Node;
struct State;
struct controls;


std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> distribution(-1.0, 1.0);

//For collision checking
Scene escena("./Models/Scenery/scenery.obj");
Object nave("./Models/Spaceship/spaceship.obj");
glm::mat4 transform_nave;

struct State{
    
    // Cuatro vectore del estado
    Vector3d p;
    Quaterniond q;
    Vector3d v;
    Vector3d w;

    // Pesos para balancear la distancia
    float wp = 0.05, wq = 5, wv = 0.3, ww = 0.5;
    //float wp = 1, wq = 1, wv = 1, ww = 1;

    State(Vector3d P, Quaterniond Q, Vector3d V, Vector3d W) : p(P), q(Q), v(V), w(W) {}

    // Fill with random data
    State(){

        while(1){
            double z = distribution(gen) * 25.0;
            double y = distribution(gen) * 8.0;
            double x = distribution(gen) * 16.0;


            p << x, y, z;
            q = Eigen::Quaterniond::Identity();
            v = Eigen::Vector3d::Random() * 5.0;
            w = Eigen::Vector3d::Random() * 2.0;

            if(!in_collision())
                return;
        }

    }


    // FUncion de distancia
    float Distance(State s2){
        float dist = wp * pow((p - s2.p).norm(), 2) + wq * pow((1 - abs( q.dot(s2.q) )), 2) 
            + wv * pow((v - s2.v).norm(), 2) + ww * pow((w - s2.w).norm(), 2);
        
        return dist;
    }

    // El nodo mas cercano del arbol al estado actual
    Node* nearest_node(Node* root);

    // Verifica colisiones con el header collisions usando CGal
    bool in_collision(){

        //Get rotation matrix
        glm::dquat glmQuaternion(
            (float)q.w(),
            (float)q.x(),
            (float)q.y(),
            (float)q.z()
        );

        glm::mat4 translation = glm::translate(glm::mat4(1.0f), glm::vec3((float)p.x(), (float)p.y(), (float)p.z()));
        glm::mat4 rotation = glm::mat4_cast(glmQuaternion);

        transform_nave = translation * rotation;
        nave.update_state(transform_nave);

        return escena.is_colliding(&nave);
    }

    // Determinar si ya terminamos
    bool finished(){
        //Queremos que quede en cierta región del espacio
        bool space = p.z() < -21.0 && abs(p.y()) < 5.0 && abs(p.x()) < 5.0; //Specific region
        bool rotation = true; 
                        //abs(q.dot(Eigen::Quaterniond::Identity())) < 0.1; //Close to identity
        return  space && rotation;
    }


};




struct Controls{
    Vector3d F;
    Vector3d T;


    Controls(){
        F << 0, 0, 0;
        T << 0, 0, 0;
    }

    Controls(Vector3d FF, Vector3d TT) : F(FF), T(TT) {}


    // Cambiar los controles a estados. Solucionamos un sistema de ecuaciones diferenciales de orden 2
    State to_state(State prev_status){
        
        Eigen::Matrix3d R = (prev_status.q).toRotationMatrix();

        Eigen::Matrix3d I; I << 2,0,0,
                                0,2,0,
                                0,0,2; //Suponemos que nuestra nave es como una esfera

        Vector3d v_punto = R * F * (0.05);
        Vector3d w_punto = R * I.inverse() * R.transpose() * T;
        
        Vector3d v_new    = prev_status.v + dt * v_punto;
        Vector3d w_new    = prev_status.w + dt * w_punto;


        Quaterniond w_gorro(0.0, w_new.x(), w_new.y(), w_new.z());
        Quaterniond q_punto((w_gorro * prev_status.q).coeffs() * 0.5);

        Vector3d p_punto = v_new;

        Vector3d p_new    = prev_status.p + dt * p_punto;
        Quaterniond q_new(prev_status.q.coeffs() + q_punto.coeffs() * dt);
        q_new.normalize();

        State new_state(p_new, q_new, v_new, w_new);
        return new_state;
    }

};

struct Node{
    Controls c;
    State s;

    Node* parent = NULL;
    std::vector<Node*> children = {};
    
    // For creating the root node
    Node(State S) : s(S), parent(nullptr){}

    Node(State S, Controls C, Node* PARENT) : s(S), c(C), parent(PARENT){}


    bool create_next_node(State random_state){
        std::vector<State> states_vec;

        float    min_dist = FLT_MAX;
        State    min_s = s;
        Controls min_c = c;

        // Itero sobre las posibles configuraciones y tomo la mas cercana al estado aleatorio
        for(Vector3d F: UF){
            for(Vector3d T: UT){
                if(distribution(gen) < 0) continue;

                Controls new_cont(F, T);
                State new_state = new_cont.to_state(s);

                if( !new_state.in_collision() ){
                    if( random_state.Distance(new_state) < min_dist){
                        min_dist = random_state.Distance(new_state);
                        min_s = new_state;
                        min_c = new_cont;
                    }
                }
            }
        }

        // Rgresas un nodo
        

        if(min_dist != FLT_MAX){
            Node* new_node = new Node(min_s, min_c, this);
            children.push_back(new_node);
            return true;
        }

        // No se creo un nuevo nodo
        return false;
    }

    // Determinar si el nodo actual ya llego al final
    bool finished(){
        return s.finished();
    }

};



Node* State::nearest_node(Node *root){    
        if(root == nullptr)
            return nullptr;

        Node* ans = root;
        float min_dist = Distance(root -> s);

        for(Node* child : root->children){
            
            Node* ans_child = nearest_node(child);
            float dist_child = Distance(ans_child -> s);

            if(dist_child < min_dist){
                ans = ans_child;
                min_dist = dist_child;
            }
        }

        return ans;
    }



Node* RRT_algorithm(Node* start, State goal, int max_iter, float step_size){
    dt = step_size;

    int total_nodes = 1;

    for(int i = 0; i < max_iter+1; i++){
        State rand_cont;

        Node* near_node = rand_cont.nearest_node(start);

        if( near_node -> create_next_node(rand_cont) ){
            Node* new_node = (near_node -> children).back();

            if( new_node -> finished() ){
                std::cout << "Expanded " <<  total_nodes << " nodes after " << i << " random nodes were sampled"  << std::endl;
                std::cout << "Final p = (" << new_node->s.p.x() << ", "
                                           << new_node->s.p.y() << ", "
                                           << new_node->s.p.z() << ")" << std::endl;;
                return new_node;
            }

            total_nodes++;
        }

        if(i % 100 == 0){
            std::cout << "Iteracion " << i << std::endl;
        }


    }

    return nullptr;
}

#endif
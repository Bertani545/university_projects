#define GLEW_STATIC

#include<iostream>
#include<GL/glew.h>
#include<GLFW/glfw3.h>

//Shaders
#include"OpenGL/shader.hpp"

//Camera
#include "OpenGL/camera.hpp"

//Meshes
#include "OpenGL/mesh.hpp"

//Moodels
#include "OpenGL/model.hpp"

//Textures
#include"OpenGL/textures.hpp"



//Matrix operations for openGL
#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>


//Matrices an vectors for RRT
#include <eigen3/Eigen/Dense> 


//Our Headers for this algorithm
#include "collisions.hpp"
#include "rrt.hpp"



//Called when the window is resized
void framebuffer_size_callback(GLFWwindow* window, int width, int height){
    glViewport(0,0, width, height);

}

//For camera
Camera camera = Camera();

float deltaTime = 0.0f; // Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame


bool render_solution = false;


//To get inputs
void processInput(GLFWwindow *window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
                
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ENTER && action == GLFW_PRESS) {
        render_solution = !render_solution;
        std::cout << "Rendering" << std::endl;
    }
}


//Mouse movement
float lastX = 0, lastY = 0;
bool firstMouse = true;

void mouse_callback(GLFWwindow* window, double xpos, double ypos){
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }
    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; 
    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);

}

//Scroll
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}



Node* get_farthest_Node(Node* root){
    if(root == nullptr)
            return nullptr;

        Node* ans = root;
        float min_dist = root -> s.p.z();

        for(Node* child : root->children){
            
            Node* ans_child = get_farthest_Node(child);
            float dist_child = ans_child -> s.p.z();

            if(dist_child < min_dist){
                ans = ans_child;
                min_dist = dist_child;
            }
        }

        return ans;
}


GLFWwindow* build_window(){
    GLFWwindow* window;

    if (!glfwInit())
        return nullptr;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(640, 480, "RRT", NULL, NULL);
    if (!window)
    {   
        std::cout << "No se pudo hacer la ventana" << std::endl;
        glfwTerminate();
        return nullptr;
    }
    //Window and what color it will have in the bacground
    glfwMakeContextCurrent(window);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glfwSwapBuffers(window);
    
    /*Did it start correctly*/
    if(glewInit() != GLEW_OK) 
        std::cout << "Glew didn't start correctly" << std::endl;
    
    

    glViewport(0,0, 640, 480); //where will openGL draw
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback); //To update this parameter

    //Add mouse detection
    glfwSetCursorPosCallback(window, mouse_callback); 

    //Add scroll
    glfwSetScrollCallback(window, scroll_callback);

    //Ad One press keys
    glfwSetKeyCallback(window, keyCallback);

    return window;
}



int main(){


    /*------------------------------------------------ RRT ------------------------------*/

    //Initial position. You can change this values
    Vector3d P(0, 0 , 19), V(0,0,0), W(0,0,0);
    Quaterniond Q(1.0, 0, 0.0, 0);

    State initial_state(P,Q,V,W);
    Node initial_node(initial_state);


    Node* path_end = RRT_algorithm(&initial_node, initial_state, 10000, 0.3);


    //No solution was found
    if(!path_end){
        std::cout << "No solution found with given parameters. Rendering the path that advanced the most" << std::endl; 
        path_end = get_farthest_Node(&initial_node);
    }else{
        std::cout << "Rendering solution" << std::endl;
    }
    std::cout << "Press enter to run" << std::endl;


    //Save path to goal or neareast node
    std::vector<Node*> spaceship_path;
    while(path_end != nullptr){
        spaceship_path.push_back(path_end);
        path_end = path_end->parent;
    } 
    std::reverse(spaceship_path.begin(), spaceship_path.end());



    /*-------------------------------------------------------------*/


    //------------------------------------ OpenGL -------------------------------------

    GLFWwindow* window = build_window();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


    // build and compile shaders
    // -------------------------
    Shader ourShader("./Shaders/model_loading.vs", "./Shaders/model_loading.fs");


    // load models
    // -----------
    Model ourModel("./Models/Scenery/scenery.obj");
    Model ourNave("./Models/Spaceship/spaceship.obj");

    


    /*---------------------- Matrices ---------------------------*/
    //Coordinate change for shader
    glm::mat4 model = glm::mat4(1.0f);

    //To transform coordinates to our camera
    glm::mat4 view;

    //Matriz de proyeccion a pantalla (Prooyeccion)
    glm::mat4 proj;



    /*---------------------------------- Printing the solution ----------------------------*/

    unsigned int path_index = 0;

    //Initial state
    glm::vec3 trans0 = {
                spaceship_path[path_index]->s.p.x(),
                spaceship_path[path_index]->s.p.y(),
                spaceship_path[path_index]->s.p.z()
            };
            glm::mat4 t0 = glm::translate(glm::mat4(1.0f), trans0);

            glm::mat4 R0 = glm::mat4_cast(glm::dquat(
                    spaceship_path[path_index]->s.q.w(),
                    spaceship_path[path_index]->s.q.x(),
                    spaceship_path[path_index]->s.q.y(),
                    spaceship_path[path_index]->s.q.z()
                ));

            
    glm::mat4 spaceship_movement = t0 * R0;


    long unsigned int frame_id = 0;

    while(!glfwWindowShouldClose(window)){


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        processInput(window);

        ourShader.use();
        
        //Change view matrix
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;  

        view = camera.GetViewMatrix();
        proj = glm::perspective(glm::radians(camera.Zoom), 800.0f / 600.0f, 0.1f, 100.0f);


        ourShader.setMat4("projection", proj);
        ourShader.setMat4("view", view);

        
        /*----------------------- Rendering the solution -------------------------------*/


        if(path_index < spaceship_path.size() && frame_id % 10 == 0 && render_solution){
            glm::vec3 trans = {
                spaceship_path[path_index]->s.p.x(),
                spaceship_path[path_index]->s.p.y(),
                spaceship_path[path_index]->s.p.z()
            };
            glm::mat4 t = glm::translate(glm::mat4(1.0f), trans);

            glm::mat4 R = glm::mat4_cast(glm::dquat(
                    spaceship_path[path_index]->s.q.w(),
                    spaceship_path[path_index]->s.q.x(),
                    spaceship_path[path_index]->s.q.y(),
                    spaceship_path[path_index]->s.q.z()
                ));

            
            spaceship_movement = t * R;
            

            path_index++;
        }
        if(path_index < spaceship_path.size() && render_solution){
            frame_id ++;
        }


        ourShader.setMat4("model", spaceship_movement);
        ourNave.Draw(ourShader);
        /*----------------------------- End of spaceship rendering -----------------------------------*/

        //Scenery
        model = glm::mat4(1.0f);
        ourShader.setMat4("model", model);
        ourModel.Draw(ourShader);
        
        //To be able to close it
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    

    //Close openGL
    glfwDestroyWindow(window);
    glfwTerminate();
    
    return 0;
}


#include <stdlib.h>
#include <iostream>
#include <GL/glew.h>
#include <GL/freeglut.h>
// Use of degrees is deprecated. Use radians for GLM functions
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include "Shader.h"
#include "Square.h"
#include "Scene.h"
#include <iomanip>

static Scene scene;
static const char* title = "Curves";
static Square canvas;

void initialize(void){
    scene.printHelp();
    glClearColor (0.0, 0.0, 0.0, 0.0); // background color
    glViewport(0,0,scene.width,scene.height);
    
    // Initialize canvas
    canvas.init( glm::vec2(0.,0.), 2.0f );

    // Initialize shader in the scene
    scene.init();

}

void display(void){
    glClear(GL_COLOR_BUFFER_BIT);
    
    
    scene.computeCurve();
    if(scene.control -> isactive){
        scene.shader -> showActive = true;
        scene.shader -> activePosition = *(scene.control -> activePoint);
    }
    else{
        scene.shader -> showActive = false;
    }
    scene.shader -> setUniforms();
    // BEGIN draw square
    canvas.draw();
    // END draw square
    
    glutSwapBuffers();
    glFlush();
}



int main(int argc, char** argv)
{
    // BEGIN CREATE WINDOW
    glutInit(&argc, argv);
    
    glutInitContextVersion(3,1);
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
    glutInitWindowSize( scene.width-1, scene.height-1 );
    glutCreateWindow(title);
    // manual reshape to hide the retina display bug
    glutReshapeWindow( scene.width, scene.height );
    glewExperimental = GL_TRUE;
    GLenum err = glewInit() ;
    if (GLEW_OK != err) {
        std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
    }
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    // END CREATE WINDOW
    
    initialize();

    glutDisplayFunc( display );
    glutKeyboardFunc( scene.keyboard );
    glutMouseFunc( scene.mouse );
    glutReshapeFunc( scene.reshape );
    glutPassiveMotionFunc( scene.passiveMotion );
    glutMotionFunc( scene.drag );
    glutMainLoop();
    
	return 0;
}

﻿///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>
#include <cmath>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "macros.h"

//Enable OpenGL drawing.  
bool drawModeEnabled = false;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

#define MAX_DEPTH 3  //number of bounces

#define CAPTION "Whitted Ray-Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1
#define BIAS 0.001
#define JITT_SAMPLES 4
#define LENS_SAMPLES 8

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];


// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float* colors;
float* vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t* img_Data;

uint8_t cells[JITT_SAMPLES * JITT_SAMPLES][2];
uint8_t cell_iterator = 0;
uint16_t samples = 1;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;

Grid* grid_ptr = NULL;
BVH* bvh_ptr = NULL;
accelerator Accel_Struct = NONE;

int RES_X, RES_Y;

int WindowHandle = 0;

// Supersampling
bool jittering = true;
bool soft_shadows = false;
bool fuzzy = false;
bool progressive =  false;
bool dof = true;
bool motion_blur = false;

float roughness = 2.0f;


/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte* errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if (isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");

	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);

	// unbind the VAO
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
	//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;
	glClear(GL_COLOR_BUFFER_BIT);

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X * RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char* filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}


// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top,
	float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

	case 27:
		glutLeaveMainLoop();
		break;

	case 'r':
		camX = Eye.x;
		camY = Eye.y;
		camZ = Eye.z;
		r = Eye.length();
		beta = asinf(camY / r) * 180.0f / 3.14f;
		alpha = atanf(camX / camZ) * 180.0f / 3.14f;
		break;

	case 'c':
		printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
		printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
		break;

	case 'j':
		jittering = !jittering;
		break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit();
	if (result != GLEW_OK) {
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	}
	GLenum err_code = glGetError();
	printf("Vendor: %s\n", glGetString(GL_VENDOR));
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("Version: %s\n", glGetString(GL_VERSION));
	printf("GLSL: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);

	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if (WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////YOUR CODE HERE///////////////////////////////////////////////////////////////////////////////////////

/*************************************************** Calculate Color ****************************************************/
Color calculateColor(Vector normal, Light* light, Vector light_dir, Vector view_dir, Material* mat, Vector pos) {
	Vector halfway_dir = (light_dir - view_dir).normalize();
	float distance = (light->position - pos).length();

	float diff = max(normal * light_dir, 0);
	Color diffuse = light->color * diff * mat->GetDiffColor() * mat->GetDiffuse();

	float spec = pow(max((normal * halfway_dir), 0.0), mat->GetShine());
	Color specular = mat->GetSpecColor() * spec * light->color * mat->GetSpecular();

	Ray r = Ray(pos, light_dir);

	if (bvh_ptr != NULL) {
		if (bvh_ptr->Traverse(r))
			return Color(0, 0, 0);
	}
	else if (grid_ptr != NULL) {
		if (grid_ptr->Traverse(r))
			return Color(0, 0, 0);
	}
	else {
		for (int i = 0; i < scene->getNumObjects(); i++) {
			Object* obj = scene->getObject(i);
			float dist = 0.0f;
			if (obj->intercepts(r, dist))
				return Color(0, 0, 0);
		}
	}
	Color c = (diffuse + specular) / (scene->getNumLights() * 0.9f);
	return c;
}
/***********************************************************************************************************************/

/************************************************ Object Intersection **************************************************/

Object* closestObject(Ray ray, float &t) {
	float minDist = INFINITY;
	Object* closest = NULL;

	for (int i = 0; i < scene->getNumObjects(); i++) {
		Object* obj = scene->getObject(i);
		float dist = 0.0f;
		if (obj->intercepts(ray, dist)) {
			if (dist < minDist) {
				closest = obj;
				minDist = dist;
			}
		}
	}
	t = minDist;
	return closest;
}
/***********************************************************************************************************************/

Color lightReflection(Vector l_pos, Vector phit, Vector normal, Vector ray_dir, Material* mat, Light* l) {
	Vector light_direction = (l_pos - phit).normalize();

	float intensity = light_direction * normal;

	Vector reflection = ray_dir - normal * (ray_dir * normal) * 2;
	reflection = reflection.normalize();

	if (intensity > 0) {
		return calculateColor(normal, l, light_direction, ray_dir, mat, phit + normal * BIAS);
	}
	return Color(0,0,0);
}

Vector chooseGridCoords(Vector pos, Light* l = nullptr, int k = 0, int j = 0) {

	if (l != nullptr && k == 0 && j == 0) {
		pos.x = pos.x - 0.5 + rand_float() * (float)l->width;
		pos.y = pos.y - 0.5 + rand_float() * (float)l->height;
	}
	else if(l != nullptr) {
		pos.x = pos.x - 0.5 + (k + rand_float()) * l->width / JITT_SAMPLES;
		pos.y = pos.y - 0.5 + (j + rand_float()) * l->height / JITT_SAMPLES;
	}
	else {
		pos.x = pos.x - 0.5 + (k + rand_float()) / JITT_SAMPLES;
		pos.y = pos.y - 0.5 + (j + rand_float()) / JITT_SAMPLES;
	}
	
	return pos;
}


/************************************************* Light Intersection **************************************************/
Color getLightContribution(Ray ray, Vector intersection_point, Vector normal, Material* mat) {
	Vector light_direction, reflection;
	Color light_contribution = Color(0, 0, 0);
	for (int i = 0; i < scene->getNumLights(); i++) {
		Light* l = scene->getLight(i);
		Vector l_pos = l->position;

		if (l->width != 0 && l->height != 0 && soft_shadows) {
			Color aux = Color(0, 0, 0);
			if (!jittering) {
				for (int k = 0; k < JITT_SAMPLES; k++) {
					for (int j = 0; j < JITT_SAMPLES; j++) {
						l_pos = chooseGridCoords(l_pos, l, k, j);

						aux += lightReflection(l_pos, intersection_point, normal, ray.direction, mat, l);
					}
				}
				aux = aux / pow(JITT_SAMPLES, 2);
				light_contribution += aux;
			}
			else {
				l_pos = chooseGridCoords(l_pos, l);

				light_contribution += lightReflection(l_pos, intersection_point, normal, ray.direction, mat, l);
			}
			
		}
		else {
			light_contribution += lightReflection(l_pos, intersection_point, normal, ray.direction, mat, l);
		}
	}
	return light_contribution;
}

float schlickApproximation(float cos_i, float n_i, float n_t) {
	float R_0 = pow(((n_i - n_t) / (n_i + n_t)), 2);
	float Kr = R_0 + (1.0f - R_0) * pow((1 - cos_i), 5);
	return Kr;
}
/***********************************************************************************************************************/


/**********************************************************************************************************************/
/*                                                RAY TRACING                                                         */
/**********************************************************************************************************************/
Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	bool inside = false;
	float minDist;
	Object* hit = NULL;
	Vector phit, nhit, L, reflection;
	Color color = Color(0, 0, 0);
	bool is_hit = false;

    /***************************/
	/*    Colision Checking    */
	/***************************/

	//If grid is active
	if (grid_ptr != NULL) {
		is_hit = grid_ptr->Traverse(ray, &hit, phit);
	}
	//If bvh is active
	else if (bvh_ptr != NULL) {
		is_hit = bvh_ptr->Traverse(ray, &hit, phit);
	}
	else {
		hit = closestObject(ray, minDist);
		if (hit != NULL) {
			//Intersection Point
			phit = ray.origin + ray.direction * minDist;
		}
	}
	
	//If ray intercepts no object return background color
	if (!is_hit && hit == NULL) return scene->GetSkyboxColor(ray);//return scene->GetBackgroundColor();

	nhit = hit->getNormal(phit);

	//If angle between normal and ray direction is above 90 degrees we are inside the object
	if (ray.direction * nhit > 0) {
		nhit = nhit * (-1);
		inside = true;
	}

	//Move intersection point up bit to avoid intersection with the same object 
	Vector offset_phit = phit + nhit * BIAS;

	//Get color due to illumination from lights
	color += getLightContribution(ray, phit, nhit, hit->GetMaterial());


	//Reflection and refraction contribution
	float Kr = 1.0f;
	if (hit->GetMaterial()->GetTransmittance() != 0 && depth < MAX_DEPTH) {
		float cos_d = - (nhit * ray.direction);
		float n = (inside) ? (hit->GetMaterial()->GetRefrIndex() / ior_1) : (ior_1 / hit->GetMaterial()->GetRefrIndex());
		float sin_refr2 = n * n * (1 - cos_d * cos_d);

		Kr = (inside) ? schlickApproximation(cos_d, hit->GetMaterial()->GetRefrIndex(), ior_1) : schlickApproximation(cos_d, ior_1, hit->GetMaterial()->GetRefrIndex());

		if (sin_refr2 <= 1) {
			float cos_refr = sqrt(1 - sin_refr2);
			Vector refraction = ray.direction * n + nhit * (n * cos_d - cos_refr);
			Ray refr_ray = Ray(phit - nhit * BIAS, refraction);
			color += rayTracing(refr_ray, depth + 1, ior_1) * hit->GetMaterial()->GetTransmittance() * (1 - Kr);
		}
		else {
			reflection = ray.direction - nhit * (ray.direction * nhit) * 2;
			Ray reflRay = Ray(offset_phit, reflection.normalize());
			color += rayTracing(reflRay, depth + 1, ior_1) * hit->GetMaterial()->GetReflection() * Kr;
		}
	}

	
	if (hit->GetMaterial()->GetReflection() > 0 && depth < MAX_DEPTH) {
		reflection = ray.direction - nhit * (ray.direction * nhit) * 2;
		if (fuzzy) {
			Vector sphere_center = offset_phit + reflection;
			Vector sphere_offset = sphere_center + rnd_unit_sphere() * 0.3f;
			Vector fuzzy_reflection = (sphere_offset - offset_phit).normalize();
			if (fuzzy_reflection * nhit > 0) reflection = fuzzy_reflection;
		}
		Ray reflRay = Ray(offset_phit, reflection.normalize());
		color += rayTracing(reflRay, depth + 1, ior_1) * hit->GetMaterial()->GetReflection() * Kr * hit->GetMaterial()->GetSpecColor();
	}

	return color;
}

/***********************************Adaptive Supersampling******************************************/
float getColorThreshold(int index) {
	int resY = scene->GetCamera()->GetResY();
	int resX = scene->GetCamera()->GetResX();

	Color pixelColor = Color(colors[index], colors[index + 1], colors[index + 2]);
	Color leftPixel = pixelColor;
	Color downPixel = pixelColor;

	if (index > 0) {
		leftPixel = Color(colors[index - 3], colors[index - 2], colors[index - 1]);
	}
	if ((index + resY * resX) < resY * resX * 3) {
		Color downPixel = Color(colors[index + resY * resX], colors[index + resY * resX + 1], colors[index + resY * resX + 1]);
	}


	float leftThreshold = abs(leftPixel.r() - pixelColor.r()) + abs(leftPixel.g() - pixelColor.g()) + abs(leftPixel.b() - pixelColor.b());
	float downThreshold = abs(downPixel.r() - pixelColor.r()) + abs(downPixel.g() - pixelColor.g()) + abs(downPixel.b() - pixelColor.b());

	return max(leftThreshold, downThreshold);
}
/*************************************************************************************************/

// Render function by primary ray casting from the eye towards the scene's objects

float lerp(float a, float b, float c) {
	return a + c * (b - a);
}

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}
	set_rand_seed(time(NULL));

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			/*************************************************************************************/
			/*                                   RAY CREATION                                    */
			/*************************************************************************************/

			Color color = Color(0, 0, 0);
			Vector pixel;  //viewport coordinates

			pixel.x = x + 0.5f;
			pixel.y = y + 0.5f;

			/*******************
			* Progressive Mode *
			*******************/
			if (progressive) {
				pixel.x = x - 0.5f + rand_float();
				pixel.y = y - 0.5f + rand_float();

				Ray ray = scene->GetCamera()->PrimaryRay(pixel);

				color += rayTracing(ray, 1, 1.0).clamp();
			}
			/*******************
			*  Jittering Mode  *
			*******************/
			else if(jittering){
				Ray ray = Ray(Vector(0,0,0), Vector(0, 0, 0));
				for (int i = 0; i < JITT_SAMPLES; i++) {
					for (int j = 0; j < JITT_SAMPLES; j++) {
						pixel = chooseGridCoords(pixel, nullptr, i, j);

						if (dof) {
							Vector lens_sample = rnd_unit_disk() * scene->GetCamera()->GetAperture();
							ray = scene->GetCamera()->PrimaryRay(lens_sample, pixel);
						}
						else if (motion_blur) {
							ray = scene->GetCamera()->PrimaryRay(pixel, rand_float());
						}
						else {
							ray = scene->GetCamera()->PrimaryRay(pixel);
						}

						color += rayTracing(ray, 1, 1.0).clamp();
					}
				}

				color = color / pow(JITT_SAMPLES, 2);
			}
			/*******************
			*   Default Mode   *
			*******************/
			else {
				Ray ray = Ray(Vector(0, 0, 0), Vector(0, 0, 0));
				if (dof) {
					for (int k = 0; k < LENS_SAMPLES; k++) {
						Vector lens_sample = rnd_unit_disk() * scene->GetCamera()->GetAperture();
						ray = scene->GetCamera()->PrimaryRay(lens_sample, pixel);
					}
				}
				else {
					ray = scene->GetCamera()->PrimaryRay(pixel);
				}
				color = rayTracing(ray, 1, 1.0).clamp();
			}
			/*************************************************************************************/
		
			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());
			
			
			
			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				if (progressive) {
					if (FrameCount == 0) {
						colors[index_col++] = (float)color.r();
						colors[index_col++] = (float)color.g();
						colors[index_col++] = (float)color.b();
					}
					else {
						colors[index_col] = lerp(colors[index_col], (float)color.r(), 1.0f/FrameCount);
						index_col++;
						colors[index_col] = lerp(colors[index_col], (float)color.g(), 1.0f/FrameCount);
						index_col++;
						colors[index_col] = lerp(colors[index_col], (float)color.b(), 1.0f/FrameCount);
						index_col++;
					}
				}
				else {
					colors[index_col++] = (float)color.r();
					colors[index_col++] = (float)color.g();

					colors[index_col++] = (float)color.b();
				}
			}
		}

	}
	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}


///////////////////////////////////////////////////////////////////////  SETUP     ///////////////////////////////////////////////////////

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}
void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
		printf("Scene loaded.\n\n");
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}


	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X * RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);

	Accel_Struct = scene->GetAccelStruct();   //Type of acceleration data structure

	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	else if (Accel_Struct == BVH_ACC) {
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();
		bvh_ptr = new BVH();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
	}
	else
		printf("No acceleration data structure.\n\n");

	unsigned int spp = scene->GetSamplesPerPixel();
	if (spp == 0)
		printf("Whitted Ray-Tracing\n");
	else
		printf("Distribution Ray-Tracing\n");

}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int
		ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();

		} while ((toupper(ch) == 'Y'));
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X * RES_Y * sizeof(float);
		size_colors = 3 * RES_X * RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
		memset(colors, 0, size_colors);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////
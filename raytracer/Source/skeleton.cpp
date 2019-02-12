#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include "limits"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

#define SCREEN_WIDTH 300
#define SCREEN_HEIGHT 300
#define FULLSCREEN_MODE false

vector<Triangle> triangles;

float focalLength = 300.0;
vec4 cameraPos( 0.0, 0.0, -3, 1.0);
vec4 lightPos( 0, -0.5, -0.7, 1.0 );
vec3 lightColor = 14.f * vec3( 1, 1, 1 );
vec3 indirectLight = 0.5f*vec3( 1, 1, 1 );
vec3 black(0.0,0.0,0.0);
mat4 R;
mat4 R_X;
mat4 R_Y;
mat4 R_Z;
float yaw = 0;
float roll = 0;
float pitch = 0;
//const vec3& tmp = vec3(0, 1, 0);


struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex;
};

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
bool ClosestIntersection(vec4 start, vec4 dir,
            const vector<Triangle>& triangles, Intersection& closestIntersection);
void UpdateCamera(const uint8_t* keystate);
void InitialiseRotation();
vec3 DirectLight( const Intersection& i, vec4 pixel );
void UpdateLightPos(const uint8_t* keystate);
//
int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  InitialiseRotation();
  LoadTestModel(triangles);
  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  Intersection closest;
  Intersection shadow;
  vec3 finalcolour;

  for(int x = 0; x < SCREEN_WIDTH; x++){
    for(int y = 0; y < SCREEN_HEIGHT; y++){
      vec4 d = vec4(x- SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength, 0);
      if (ClosestIntersection(cameraPos, d, triangles, closest)) {
        vec3 directLight = DirectLight(closest, d);
        finalcolour = triangles[closest.triangleIndex].color*(directLight+indirectLight);
        PutPixelSDL(screen, x, y, finalcolour);
      }
      else
        PutPixelSDL(screen,x,y,black);
    }
  }
}

/*Place updates of parameters here*/
void Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  const uint8_t* keystate = SDL_GetKeyboardState( 0 );
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
//  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
  UpdateCamera(keystate);
  UpdateLightPos(keystate);
}

// finds if intersection exists between light and object to camera
// outputs true if intersection found
bool ClosestIntersection(vec4 start, vec4 dir,
            const vector<Triangle>& triangles, Intersection& closestIntersection)
{
  bool intersect = false;
  closestIntersection.distance = std::numeric_limits<float>::max();

  // runs through all triangles
  for(uint32_t i = 0; i < triangles.size(); i++){
    Triangle triangle = triangles[i];
    vec4 v0 = triangle.v0;
    vec4 v1 = triangle.v1;
    vec4 v2 = triangle.v2;
    vec3 e1 = vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x,start.y-v0.y,start.z-v0.z);
    mat3 A = mat3( -vec3(dir.x,dir.y,dir.z), e1, e2 );
    vec3 x = glm::inverse( A ) * b;

    float t = x.x;
    float u = x.y;
    float v = x.z;
  //  if (t < 0) {
  //    intersect = false;
  //    break;
  //  }

    // checks if intersection found is closer to previous intersection (if exists)
    float distance = glm::length(t*dir);
    if (u >= 0 && v >= 0 && (u+v) <= 1 && t >= 0) {
      if (distance < closestIntersection.distance) {
        closestIntersection.position = start + (t*dir);
        closestIntersection.distance = distance;
        closestIntersection.triangleIndex = i;
  //      printf("%f",distance);
        intersect = true;
      }
    }
  }
  return intersect;
}

void UpdateCamera(const uint8_t* keystate) {

//  vec3 to = vec3(0,0,0);
//  mat4 R = lookAt(vec3(cameraPos.x, cameraPos.y,cameraPos.z), to);
  vec4 right(R[0][0], R[0][1], R[0][2], 1 );
  vec4 down(R[1][0], R[1][1], R[1][2], 1 );
  vec4 forward( R[2][0], R[2][1], R[2][2], 1 );

  if( keystate[SDL_SCANCODE_UP] )
  {
  // Move camera forward
    cameraPos += forward;
    printf("forward\n");
  }
  if( keystate[SDL_SCANCODE_DOWN] )
  {
  // Move camera backward
    cameraPos -= forward;
    printf("back\n");
  }
  if( keystate[SDL_SCANCODE_LEFT] )
  {
  // Move camera to the left
    cameraPos -= right;
    printf("left\n");
  }
  if( keystate[SDL_SCANCODE_RIGHT] )
  {
  // Move camera to the right
    cameraPos += right;
    printf("right\n");
  }
}

void InitialiseRotation(){
  R_X[0] = vec4(1,0,0,0);
  R_X[1] = vec4(0,cos(pitch),-sin(pitch),0);
  R_X[2] = vec4(0,sin(pitch),cos(pitch),0);
  R_X[3] = vec4(0,0,0,1);
  R_Y[0] = vec4(cos(yaw),0,sin(yaw),0);
  R_Y[1] = vec4(0,1,0,0);
  R_Y[2] = vec4(-sin(yaw),0,cos(yaw),0);
  R_Y[3] = vec4(0,0,0,1);
  R_Z[0] = vec4(cos(roll),-sin(roll),0,0);
  R_Z[1] = vec4(sin(roll),cos(roll),0,0);
  R_Z[2] = vec4(0,0,1,0);
  R_Z[3] = vec4(0,0,0,1);
  R = R_X * R_Y * R_Z;
}

// calculates the direct light
vec3 DirectLight( const Intersection& i, vec4 pixel ) {
  Intersection shadows;
  vec3 directlight(0,0,0);
  vec4 r = lightPos - i.position;
  vec4 n = glm::normalize(triangles[i.triangleIndex].normal);
  float lightDistance = glm::length(r);
  r = glm::normalize(r);
  float area = 4 * M_PI * lightDistance * lightDistance;
  float max_scalar = max(glm::dot(r,n),0.0f) / area;
  if(ClosestIntersection(i.position+r*0.00001f, r, triangles, shadows)) {
    if (lightDistance > shadows.distance) max_scalar = 0;
  }
  directlight = lightColor*max_scalar;
  return directlight;
}

void UpdateLightPos(const uint8_t *keystate){
  vec4 right(0.1, 0, 0, 1 );
  vec4 down(0, 0.1, 0, 1 );
  vec4 forward( 0, 0, 0.1, 1 );
  if( keystate[SDL_SCANCODE_W] )
  {
    lightPos = lightPos+forward;
    // Move camera forward
  }
  if( keystate[SDL_SCANCODE_S] )
  {
    lightPos=lightPos-forward;
    // Move camera backward
  }
  if( keystate[SDL_SCANCODE_A] )
  {
    lightPos= lightPos-right;
    // Move camera to the left
  }
  if( keystate[SDL_SCANCODE_D] )
  {
    lightPos= lightPos+right;
    // Move camera to the right
  }

}

// mat4 lookAt(const vec3& from, const vec3& to, const vec3& tmp)
// {
//   vec3 forward = glm::normalize(from - to);
//   vec3 right = glm::cross(glm::normalize(tmp), forward);
//   vec3 up = glm::cross(forward, right);
//
//   mat4 camToWorld;
//
//   camToWorld[0][0] = right.x;
//   camToWorld[0][1] = right.y;
//   camToWorld[0][2] = right.z;
//   camToWorld[1][0] = up.x;
//   camToWorld[1][1] = up.y;
//   camToWorld[1][2] = up.z;
//   camToWorld[2][0] = forward.x;
//   camToWorld[2][1] = forward.y;
//   camToWorld[2][2] = forward.z;
//
//   camToWorld[3][0] = from.x;
//   camToWorld[3][1] = from.y;
//   camToWorld[3][2] = from.z;
//
//   return camToWorld;
// }

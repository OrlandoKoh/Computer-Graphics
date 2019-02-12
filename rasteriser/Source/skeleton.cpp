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
using glm::ivec2;
using glm::vec2;

#define SCREEN_WIDTH 500
#define SCREEN_HEIGHT 500
#define FULLSCREEN_MODE false

struct Pixel
{
  int x;
  int y;
  float zinv;
  vec4 pos3d;
};

struct Vertex
{
  vec4 position;
};

vector<Triangle> triangles;

float focalLength = 500.0;
vec4 cameraPos(0.0, 0.0, -3.001, 1.0);
vec4 lightPos(0,-0.5,-0.7, 1);
vec3 lightPower = 14.f*vec3( 1, 1, 1 );
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );
vec3 black(0.0,0.0,0.0);
mat4 R;
mat4 R_X;
mat4 R_Y;
mat4 R_Z;
mat4 C1;
mat4 C2;
mat4 transform;
float yaw = 0;
float roll = 0;
float pitch = 0;
vector<Pixel> leftPixels( SCREEN_HEIGHT );
vector<Pixel> rightPixels( SCREEN_HEIGHT );
vec3 currentColor;
vec4 currentNormal;
float currentReflectance;
screen *surface;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

//const vec3& tmp = vec3(0, 1, 0);

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void UpdateCamera(const uint8_t* keystate);
void InitialiseRotation();
void VertexShader( const Vertex& v, Pixel& p );
void UpdateLightPos(const uint8_t* keystate);
void TransformationMatrix();
void DrawLineSDL( screen* surface, Pixel a, Pixel b, vec3 color );
void DrawPolygonEdges( const vector<vec4>& vertices, screen *screen);
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void ComputePolygonRows( const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels,
                    vector<Pixel>& rightPixels );
void DrawPolygonRows( const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels);
void DrawPolygon( const vector<Vertex>& vertices );
void PixelShader( const Pixel& p );

//
int main( int argc, char* argv[] )
{

  surface = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  InitialiseRotation();
  TransformationMatrix();
  LoadTestModel(triangles);
  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(surface);
      SDL_Renderframe(surface);
    }

  SDL_SaveImage( surface, "screenshot.bmp" );

  KillSDL(surface);
  return 0;
}


/*Place your drawing here*/
void Draw(screen *screen)
{
  for( int y=0; y<SCREEN_HEIGHT; ++y )
    for( int x=0; x<SCREEN_WIDTH; ++x )
      depthBuffer[y][x] = 0;
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  for( uint32_t i=0; i<triangles.size(); ++i )
  {
    vector<Vertex> vertices(3);
    vertices[0].position = triangles[i].v0;
    vertices[1].position = triangles[i].v1;
    vertices[2].position = triangles[i].v2;
    currentColor = triangles[i].color;
    currentNormal = triangles[i].normal;
    currentReflectance = 1;

    DrawPolygon( vertices );
    // for(int v=0; v<3; ++v)
    // {
    //   Pixel projPos;
    //   VertexShader( vertices[v], projPos );
    //   vec3 color(1,1,1);
    //   PutPixelSDL( screen, projPos.x, projPos.y, color );
    // }
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

// extracts individual pixels from vertices
void VertexShader( const Vertex& v, Pixel& p ){
  vec4 proj = v.position - cameraPos;
  p.zinv = 1/proj.z;
  p.x = int(focalLength*(proj.x * 1/proj.z) + SCREEN_WIDTH / 2);
  p.y = int(focalLength*(proj.y * 1/proj.z) + SCREEN_HEIGHT / 2);
  p.pos3d = v.position;

  PixelShader(p);
}

// Shades each individual pixel and draws it
void PixelShader( const Pixel& p )
{
  int x = p.x;
  int y = p.y;
  vec3 normal = vec3(currentNormal.x, currentNormal.y, currentNormal.z);
  if( p.zinv > depthBuffer[y][x] )
  {
    depthBuffer[y][x] = p.zinv;
    vec4 r = lightPos - p.pos3d;
    vec3 r3 = vec3(r.x,r.y,r.z);
    float dist = glm::length(r3);
    vec3 light_area = lightPower/(float)(4*M_PI *dist *dist);
    vec3 light = light_area * max(glm::dot(glm::normalize(normal),glm::normalize(r3)),0.f);
    vec3 color = (light+indirectLightPowerPerArea)*currentColor;
    PutPixelSDL( surface, x, y, color );
  }
}

// goes through every pixel and calls pixel shader to draw it
void DrawLineSDL( screen* surface, Pixel a, Pixel b, vec3 color ) {
  ivec2 delta = glm::abs( ivec2(a.x,a.y) - ivec2(b.x,b.y) );
  int pixels = glm::max( delta.x, delta.y ) + 1;
  vector<Pixel> result(pixels);
  Interpolate(a,b,result);

  for(int v=0; v < int(result.size()); ++v)
  {
    Pixel current = result[v];
    PixelShader(current);
  }
}

void DrawPolygonEdges( const vector<Vertex>& vertices, screen *screen )
{
  int V = vertices.size();

  // Transform each vertex from 3D world position to 2D image position:
  vector<Pixel> projectedVertices( V );
  for( int i=0; i<V; ++i )
  {
    VertexShader( vertices[i], projectedVertices[i] );
  }

  // Loop over all vertices and draw the edge from it to the next vertex:
  for( int i=0; i<V; ++i )
  {
    int j = (i+1)%V; // The next vertex
    vec3 color( 1, 1, 1 );
    DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
  }
}

// Interpolate between edges of an object in the scene
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result )
{
  int N = result.size();
  float initz = a.zinv;
  float initx = a.x;
  float inity = a.y;
  vec4 initp = a.pos3d*a.zinv;
  //vec2 step = vec2(b.x,b.y)-vec2(a.x,a.y) / float(max(N-1,1));
//  vec2 current( vec2(a.x,a.y) );
  float xstep = (b.x - a.x) /float (max(N-1,1));
  float ystep = (b.y - a.y) /float (max(N-1,1));
  float zstep = (b.zinv-a.zinv)/float(max(N-1,1));
  vec4 pstep = (b.pos3d*a.zinv-a.pos3d*a.zinv)/float(max(N-1,1));

  for( int i=0; i<N; ++i )
  {
    result[i].x = initx;
    result[i].y = inity;
    result[i].zinv = initz;
    result[i].pos3d = initp/a.zinv;
    initx += xstep;
    inity += ystep;
    initz += zstep;
    initp += pstep;
  }
}

void ComputePolygonRows( const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels,
                    vector<Pixel>& rightPixels )
{
// 1. Find max and min y-value of the polygon
//and compute the number of rows it occupies.
  int V = vertexPixels.size();
  int maxy = 0;
  int miny = +numeric_limits<int>::max();
  for( int i=0; i<V; i++ )
  {
    if(vertexPixels[i].y < miny)  miny = vertexPixels[i].y ;
    if(vertexPixels[i].y > maxy)  maxy = vertexPixels[i].y ;
  }

  int ROWS = maxy - miny + 1;

// 2. Resize leftPixels and rightPixels
//so that they have an element for each row.

  leftPixels = vector<Pixel>(ROWS);
  rightPixels = vector<Pixel>(ROWS);

// 3. Initialize the x-coordinates in leftPixels
//to some really large value and the x-coordinates
//in rightPixels to some really small value.
  for( int i=0; i<ROWS; i++ )
  {
    leftPixels[i].x = +numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
  }
// 4. Loop through all edges of the polygon and use
//linear interpolation to find the x-coordinate for
//each row it occupies. Update the corresponding
//values in rightPixels and leftPixels.
  vector<Pixel> result(ROWS);

  for( int i=0; i<V; ++i )
  {
    int j = (i+1)%V; // The next vertex
    Interpolate( vertexPixels[i], vertexPixels[j], result );

    for(int v=0; v < int(result.size()); ++v)
    {
      int currenty = result[v].y%ROWS;
      if(leftPixels[currenty].x > result[v].x )  leftPixels[currenty] = result[v];
      if(rightPixels[currenty].x < result[v].x)  rightPixels[currenty] = result[v];
    }
  }
}

// go through all rows by drawing lines for every row
void DrawPolygonRows( const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels) {
  int rows = leftPixels.size();
  for (int i = 0; i < rows; i++)
  {
    DrawLineSDL( surface, leftPixels[i], rightPixels[i], currentColor);
  }
}

// uses vertices that is given to use compute function to provide data needed for draw row functions
void DrawPolygon( const vector<Vertex>& vertices )
{
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i )  VertexShader( vertices[i], vertexPixels[i] );

  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  DrawPolygonRows( leftPixels, rightPixels );
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

void TransformationMatrix() {

  C1[0] = vec4(0,0,0,cameraPos.x);
  C1[1] = vec4(0,0,0,cameraPos.y);
  C1[2] = vec4(0,0,0,cameraPos.z);
  C1[3] = vec4(0,0,0,1);
  C2[0] = vec4(0,0,0,-cameraPos.x);
  C2[1] = vec4(0,0,0,-cameraPos.y);
  C2[2] = vec4(0,0,0,-cameraPos.z);
  C2[3] = vec4(0,0,0,1);

  transform = C1 * R * C2;

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

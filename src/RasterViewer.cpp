#include "SDLViewer.h"
#include <Eigen/Core>
#include <unistd.h>
#include <functional>
#include <iostream>

#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

char mode;
char method;
int cnt = 0;
bool translate = false;

Vector4f prevColor1(1,0,0,1);
Vector4f prevColor2(1,0,0,1);
Vector4f prevColor3(1,0,0,1);

bool numpad = false;
bool Recording = false;
int cntTimes = 0;
int clicked_triangle = -1;
int clicked_vertex = -1;
float trans_x = 0;
float trans_y = 0;
float trans_move_x = 0;
float trans_move_y = 0;
float startingFrame[6], endingFrame[6], middle1Frame[6],middle2Frame[6];
float x1_o = 0;
float y1_o = 0;
float x2_o = 0;
float y2_o = 0;
float x3_o = 0;
float y3_o = 0;
double xc;
double yc;
float triCoor[6];
// Used to find the vertex nearest to the cursor
int find_nearest_vertex(Vector4f cursor, vector<VertexAttributes> vertices){
    float nearest = 999999;
    int index = 0;
    for(int i=0;i<vertices.size();i++){
        Vector4f move_val(vertices[i].move_x, vertices[i].move_y, 0,0);
        float dist = (vertices[i].position + move_val - cursor).norm();
        if(dist < nearest){
            nearest = dist;
            index = i;
        }
    }
    return index;
}

// Used to find the triangle nearest to cursor
int find_nearest_triangle(Vector4f cursor, vector<VertexAttributes> vertices){
    float nearest = 10;
    int index = -3;
    for(int i=0;i<vertices.size();i++){
        Vector4f move_val(vertices[i].move_x, vertices[i].move_y, 0,0);
        float dist = (vertices[i].position + move_val - cursor).norm();
        if(dist < nearest){
            nearest = dist;
            index = i;
        }
    }
    return index/3;
}
void initVertex(vector<VertexAttributes>& vertices,int x,int y, int width, int height,const UniformAttributes& uniform){
    vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                    ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
    
}

void printVertex(vector<VertexAttributes> vertices){
    for(int i=0;i<vertices.size();i++){
         printf("(%f,%f)\n",vertices[i].position[0],vertices[i].position[1]);
    }
}

float linear_interpolation(float a,float b,float x ){
    return a + (b-a)*x;
}

float Bezier(float a, float b,float c, float d,float u){
    
    return pow(1-u,3)*a+3*u*pow(1-u,2)*b+3*pow(u,2)*(1-u)*c+pow(u,3)*d;
}
int nChoosek(int n, int k){
    if (k == 0){
        return 1;
    }
        return (n*nChoosek(n-1,k-1))/k;
}

int main(int argc, char *args[])
{
    int width = 2000;
    int height = 2000;
	Matrix<FrameBufferAttributes,Dynamic,Dynamic> frameBuffer(width, height);

	UniformAttributes uniform;
    
    uniform.view <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

	Program program;

    // Rasterizez lines that encapsulates each triangle
    Program edges;

    // The vertex shader is the identity
	edges.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{   
        VertexAttributes out = va;

        // translation in vertex shader
        out.position[0] += out.move_x;
        out.position[1] += out.move_y;

        // If the vertex needs to be rotated w.r.t. centroid
        if(out.degree != 0){
            float rot_x0 = out.centroid[0];
            float rot_y0 = out.centroid[1];
            float rot_x1 = out.position[0];
            float rot_y1 = out.position[1];
            float pi = 2*acos(0.0);
            float theta = out.degree * (pi / 18);
            out.position[0] = (rot_x1 - rot_x0)*cos(theta) + (rot_y1 - rot_y0)*sin(theta) + rot_x0;
            out.position[1] = -(rot_x1 - rot_x0)*sin(theta) + (rot_y1 - rot_y0)*cos(theta) + rot_y0;
        }

        // If the vertex needs to be scaled w.r.t. centroid
        if(out.scale != 1){
            float scl_x0 = out.centroid[0];
            float scl_y0 = out.centroid[1];
            float scl_x1 = out.position[0];
            float scl_y1 = out.position[1];
            float mult = pow(1.25,out.scale);
            out.position[0] = (scl_x1 - scl_x0)*mult + scl_x0;
            out.position[1] = (scl_y1 - scl_y0)*mult + scl_y0;
        }

        // For view control
        Vector4f finalView = uniform.view * out.position;
        out.position << finalView(0),finalView(1),finalView(2),finalView(3);
        out.color<<1,1,1,1;

		return out;
	};

	// The fragment shader uses a fixed color
	edges.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(va.color(0),va.color(1),va.color(2));
	};

	// The blending shader converts colors between 0 and 1 to uint8
	edges.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{   
        VertexAttributes out = va;
        
        // translation in vertex shader
        out.position[0] += out.move_x;
        out.position[1] += out.move_y;
        
        // If the vertex needs to be rotated w.r.t. centroid
        if(out.degree != 0){
            float rot_x0 = out.centroid[0];
            float rot_y0 = out.centroid[1];
            float rot_x1 = out.position[0];
            float rot_y1 = out.position[1];
            float pi = 2*acos(0.0);
            float theta = out.degree * (pi / 18);
            out.position[0] = (rot_x1 - rot_x0)*cos(theta) + (rot_y1 - rot_y0)*sin(theta) + rot_x0;
            out.position[1] = -(rot_x1 - rot_x0)*sin(theta) + (rot_y1 - rot_y0)*cos(theta) + rot_y0;
        }

        // If the vertex needs to be scaled w.r.t. centroid
        if(out.scale != 1){
            float scl_x0 = out.centroid[0];
            float scl_y0 = out.centroid[1];
            float scl_x1 = out.position[0];
            float scl_y1 = out.position[1];
            float mult = pow(1.25,out.scale);
            out.position[0] = (scl_x1 - scl_x0)*mult + scl_x0;
            out.position[1] = (scl_y1 - scl_y0)*mult + scl_y0;
        }

        // For view control
        Vector4f finalView = uniform.view * out.position;
        out.position << finalView(0),finalView(1),finalView(2),finalView(3);
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(va.color(0),va.color(1),va.color(2));
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	// One triangle in the center of the screen
	vector<VertexAttributes> vertices;

    // Temp data structure to draw intermediate stages of drawing a triangle
    vector<VertexAttributes> insertion_vertices;
    vector<VertexAttributes> temp;
    //Data structures for animation
    vector<vector<int>> key_rot;
    vector<vector<float>> key_scl;
    vector<vector<float>> key_posx;
    vector<vector<float>> key_posy;

    // Initialize the viewer and the corresponding callbacks
    SDLViewer viewer;
    viewer.init("Viewer Example", width, height);

    viewer.mouse_move = [&](int x, int y, int xrel, int yrel){
        // Action when in mode == 'i'ert mode
        if(mode == 'i'){
            if(insertion_vertices.size()==cnt){
                initVertex(insertion_vertices,x,y,width,height,uniform);
                insertion_vertices[insertion_vertices.size()-1].color << 0,0,1,1;
            }
            else{
                insertion_vertices[insertion_vertices.size()-1].position << ((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1;
                insertion_vertices[insertion_vertices.size()-1].color << 0,0,1,1;
            }
            
            viewer.redraw_next = true;
           
   
        }

        else if(mode == 'o'){
            if(translate){
                Vector4f cursor(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1);
                trans_move_x = cursor(0) - trans_x;
                trans_move_y = cursor(1) - trans_y;
                
                // Initial implementation by changing postion of vertices
                //vertices[clicked_triangle*3+0].position[0] = x1_o + trans_move_x;
                //vertices[clicked_triangle*3+0].position[1] = y1_o + trans_move_y;
                //vertices[clicked_triangle*3+1].position[0] = x2_o + trans_move_x;
                //vertices[clicked_triangle*3+1].position[1] = y2_o + trans_move_y;
                //vertices[clicked_triangle*3+2].position[0] = x3_o + trans_move_x;
                //vertices[clicked_triangle*3+2].position[1] = y3_o + trans_move_y;

                for(int i=0;i<3;i++){
                  vertices[clicked_triangle*3+i].move_x = triCoor[2*i] + trans_move_x; 
                  vertices[clicked_triangle*3+i].move_y = triCoor[2*i+1] + trans_move_y; 
                }
                

                Vector4f centroid(0,0,0,0);
                for (int k = 0; k < 3; k++) {
                    Vector4f move_val(vertices[clicked_triangle*3+k].move_x, vertices[clicked_triangle*3+k].move_y, 0,0);
			        centroid += vertices[clicked_triangle*3+k].position + move_val;
		        }
                centroid /= 3;
                vertices[clicked_triangle*3+0].centroid << centroid;
                vertices[clicked_triangle*3+1].centroid << centroid;
                vertices[clicked_triangle*3+2].centroid << centroid;

                vertices[clicked_triangle*3+0].color << 0,0,1,1;
                vertices[clicked_triangle*3+1].color << 0,0,1,1;
                vertices[clicked_triangle*3+2].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
        }
    };

    viewer.mouse_pressed = [&](int x, int y, bool is_pressed, int button, int clicks) {
        
        //Actions when in mode == 'i'ert mode
        if(mode == 'i'){
            if(!is_pressed){
                if(cnt==0){
                cnt++;
                insertion_vertices.clear();
                initVertex(vertices,x,y,width,height,uniform);
                initVertex(insertion_vertices,x,y,width,height,uniform);
                insertion_vertices[insertion_vertices.size()-1].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
            else if(cnt==1){
                cnt++;
                initVertex(vertices,x,y,width,height,uniform);
                initVertex(insertion_vertices,x,y,width,height,uniform);
                insertion_vertices[insertion_vertices.size()-1].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
            else if(cnt==2){
                vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
                vertices[vertices.size()-1].color << 1,0,0,1;
                Vector4f centroid(0,0,0,0);
                for (int k = 1; k < 4; k++) {
			        centroid += vertices[vertices.size()-k].position;
		        }
                centroid /= 3;
                vertices[vertices.size()-1].centroid << centroid;
                vertices[vertices.size()-2].centroid << centroid;
                vertices[vertices.size()-3].centroid << centroid;

                viewer.redraw_next = true;
                cnt=0;
            }
            }
            
        }
        else if(mode == 'o'){
            if(is_pressed){
                Vector4f cursor(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1);
                clicked_triangle = find_nearest_triangle(cursor, vertices);
                if(Recording&&!cntTimes){
                    for (int k = 0; k < 3; k++) {
                        startingFrame[2*k] = vertices[clicked_triangle*3+k].position[0];
                        startingFrame[2*k+1] = vertices[clicked_triangle*3+k].position[1];
		             }
                     cntTimes++;
                }
                trans_x = cursor(0);
                trans_y = cursor(1);
                for(int i=0;i<3;i++){
                    triCoor[2*i] = vertices[clicked_triangle*3+i].move_x;
                    triCoor[2*i+1] = vertices[clicked_triangle*3+i].move_y;
                }

                prevColor1 << vertices[clicked_triangle*3+0].color;
                prevColor2 << vertices[clicked_triangle*3+1].color;
                prevColor3 << vertices[clicked_triangle*3+2].color;

                translate = true;
            }
            else{
                translate = false;
                if(Recording){
                    cout<<cntTimes<<endl;
                    if(cntTimes==1){
                        for (int k = 0; k < 3; k++) {
                            middle1Frame[2*k] = vertices[clicked_triangle*3+k].position[0]+vertices[clicked_triangle*3+k].move_x;
                            middle1Frame[2*k+1] = vertices[clicked_triangle*3+k].position[1]+vertices[clicked_triangle*3+k].move_y;
		                }
       
                        cntTimes++;
                        printf("1");
                    }else if (cntTimes==2){
                       for (int k = 0; k < 3; k++) {
                            middle2Frame[2*k] = vertices[clicked_triangle*3+k].position[0]+vertices[clicked_triangle*3+k].move_x;
                            middle2Frame[2*k+1] = vertices[clicked_triangle*3+k].position[1]+vertices[clicked_triangle*3+k].move_y;
		                }
       
                        cntTimes++;
                    }
                    else if(cntTimes==3){
                         for (int k = 0; k < 3; k++) {
                            endingFrame[2*k] = vertices[clicked_triangle*3+k].position[0]+vertices[clicked_triangle*3+k].move_x;
                            endingFrame[2*k+1] = vertices[clicked_triangle*3+k].position[1]+vertices[clicked_triangle*3+k].move_y;
		                }
        
                        cntTimes++;
                    }

                //
                if(cntTimes==4){
                    for(int i=0;i<3;i++){
                    temp.push_back(VertexAttributes(startingFrame[2*i],startingFrame[2*i+1],0,1));
                     }
                }
                

                printf("\n");
                if(method == 'l' && cntTimes==4){
                        
                    for(float t = 0.1; t < 1.00; t+=0.01){
            
                    for(int i = 0; i < 3; i++){
                        xc=(linear_interpolation(startingFrame[2*i],endingFrame[2*i],t));
                        yc=(linear_interpolation(startingFrame[2*i+1],endingFrame[2*i+1],t));
                        printf("(%f,%f)",xc,yc);
                        temp[i].position << xc, yc, 0, 1;
                    }
                    printVertex(temp);
                    printf("-------------");
                    viewer.redraw(viewer);
                    usleep(10000);
                    }
                }
                    if(method == 'b' && cntTimes==4){
                        for(float t = 0.1; t < 1.00; t+=0.01){
            
                    for(int i = 0; i < 3; i++){
                        xc=(Bezier(startingFrame[2*i],middle1Frame[2*i],middle2Frame[2*i],endingFrame[2*i],t));
                        yc=(Bezier(startingFrame[2*i+1],middle1Frame[2*i+1],middle2Frame[2*i+1],endingFrame[2*i+1],t));
                       
                        printf("(%f,%f)",xc,yc);
                        temp[i].position << xc, yc, 0, 1;
                    }
                    printVertex(temp);
                    printf("-------------");
                    viewer.redraw(viewer);
                    usleep(10000);
                    }




                }
                    if(cntTimes==4){
                        cntTimes=0;
                        Recording = false;
                        temp.clear();
                    }
                    
                    
                }

                Vector4f centroid(0,0,0,0);
                for (int k = 0; k < 3; k++) {
                    Vector4f move_val(vertices[clicked_triangle*3+k].move_x, vertices[clicked_triangle*3+k].move_y, 0,0);
			        centroid += vertices[clicked_triangle*3+k].position + move_val;
		        }
                centroid /= 3;
                vertices[clicked_triangle*3+0].centroid << centroid;
                vertices[clicked_triangle*3+1].centroid << centroid;
                vertices[clicked_triangle*3+2].centroid << centroid;

                vertices[clicked_triangle*3+0].color << prevColor1;
                vertices[clicked_triangle*3+1].color << prevColor2;
                vertices[clicked_triangle*3+2].color << prevColor3;
                viewer.redraw_next = true;




            }
        }

        else if(mode == 'p'){
            if(is_pressed){
                Vector4f cursor(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1);
                clicked_triangle = find_nearest_triangle(cursor, vertices);
                vertices[clicked_triangle*3+0].color << 0,0,1,1;
                vertices[clicked_triangle*3+1].color << 0,0,1,1;
                vertices[clicked_triangle*3+2].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
            else{
                vertices.erase(vertices.begin()+clicked_triangle*3+0,vertices.begin()+clicked_triangle*3+3);
                viewer.redraw_next = true;
            }
        }

        else if(mode == 'c'){
            if(!is_pressed){
                Vector4f cursor(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1);
                clicked_vertex = find_nearest_vertex(cursor, vertices);
                vertices[clicked_vertex].color << 0,0,1,1;
                viewer.redraw_next = true;
                numpad = true;
            }
        }
    };

    viewer.mouse_wheel = [&](int dx, int dy, bool is_direction_normal) {
    };

    viewer.key_pressed = [&](char key, bool is_pressed, int modifier, int repeat) {
        
        // Events/Action needed when different keys are pressed
        switch (key)
        {
        case 'i':
            mode = 'i';
            break;
        case 'o':
            mode = 'o';
            break;
        case 'p':
            mode = 'p';
            break;
        case 'c':
            mode = 'c';
            break;
        case '1':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color << 0.3,0.5,0.4,1;
                viewer.redraw_next = true;
            }
            break;
        case '2':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color << 0.1,0.5,0.2,1;
                viewer.redraw_next = true;
            }
            break;
        case '3':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color << 0.36,0.43,0.56,1;
                viewer.redraw_next = true;
            }
            break;
        case '4':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color << 0.63,0.26,0.157,1;
                viewer.redraw_next = true;
            }
            break;
        case '5':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color << 0.78,0.45,0.26,1;
                viewer.redraw_next = true;
            }
            break;
        case '6':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color << 0.96,0.57,0.23,1;
                viewer.redraw_next = true;
            }
            break;
        case '7':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color <<  0.69,0.45,0.1,1;
                viewer.redraw_next = true;
            }
            break;
        case '8':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color << 0.81,0.71,0.51,1;
                viewer.redraw_next = true;
            }
            break;
        case '9':
            if(!is_pressed && mode == 'c' && numpad){
                vertices[clicked_vertex].color << 0.53,0.72,0.43,1;
                viewer.redraw_next = true;
            }
            break;
        case 'h':
            if(!is_pressed && mode == 'o' && clicked_triangle > -1){
                vertices[clicked_triangle*3+0].degree++;
                vertices[clicked_triangle*3+1].degree++;
                vertices[clicked_triangle*3+2].degree++;
                viewer.redraw_next = true;
            }
            break;
        case 'j':
            if(!is_pressed && mode == 'o' && clicked_triangle > -1){
                vertices[clicked_triangle*3+0].degree--;
                vertices[clicked_triangle*3+1].degree--;
                vertices[clicked_triangle*3+2].degree--;
                viewer.redraw_next = true;
            }
            break;
        case 'k':
            if(!is_pressed && mode == 'o' && clicked_triangle > -1){
                vertices[clicked_triangle*3+0].scale++;
                vertices[clicked_triangle*3+1].scale++;
                vertices[clicked_triangle*3+2].scale++;
                viewer.redraw_next = true;
            }
            break;
        case 'l':
            if(!is_pressed && mode == 'o' && clicked_triangle > -1){
                vertices[clicked_triangle*3+0].scale--;
                vertices[clicked_triangle*3+1].scale--;
                vertices[clicked_triangle*3+2].scale--;
                viewer.redraw_next = true;
            }
            break;
        case '='://+
            if(!is_pressed){
                uniform.view(0,0) *= 1.2;
                uniform.view(1,1) *= 1.2;
                viewer.redraw_next = true;
            }
            break;
        case '+':
            if(!is_pressed){
                uniform.view(0,0) *= 1.2;
                uniform.view(1,1) *= 1.2;
                viewer.redraw_next = true;
            }
            break;
        case '-':
            if(!is_pressed){
                uniform.view(0,0) *= 0.8;
                uniform.view(1,1) *= 0.8;
                viewer.redraw_next = true;
            }
            break;
        case 's':
            if(!is_pressed){
                uniform.view(1,3) += 0.2;
                viewer.redraw_next = true;
            }
            break;
        case 'd':
            if(!is_pressed){
                uniform.view(0,3) -= 0.2;
                viewer.redraw_next = true;
            }
            break;
        case 'w':
            if(!is_pressed){
                uniform.view(1,3) -= 0.2;
                viewer.redraw_next = true;
            }
            break;
        case 'a':
            if(!is_pressed){
                uniform.view(0,3) += 0.2;
                viewer.redraw_next = true;
            }
            break;
        case 'f':
            Recording = true;
            method = 'l';  

        break;

        case 'g':
            Recording = true;
            method = 'b';
        break;
        
        default:
            break;
        }
    };

    viewer.redraw = [&](SDLViewer &viewer) {
        // Clear the framebuffer
        for (unsigned i=0;i<frameBuffer.rows();i++)
            for (unsigned j=0;j<frameBuffer.cols();j++)
                frameBuffer(i,j).color << 0,0,0,1;
        if(mode == 'i'){
            if(cnt==1){
                rasterize_lines(program,uniform,insertion_vertices,1,frameBuffer);
            }
            else if(cnt==2){
                rasterize_triangles(program,uniform,insertion_vertices,frameBuffer);
            }
        }
        vector<VertexAttributes> lines;
        for(int i=0;i<vertices.size()/3;i++){
            lines.push_back(vertices[i*3+0]);
            lines.push_back(vertices[i*3+1]);
            lines.push_back(vertices[i*3+1]);
            lines.push_back(vertices[i*3+2]);
            lines.push_back(vertices[i*3+2]);
            lines.push_back(vertices[i*3+0]);
        }
        rasterize_triangles(program,uniform,vertices,frameBuffer);
        rasterize_lines(edges,uniform,lines,1,frameBuffer);
       	
        if(method=='b' || method=='l'){
            rasterize_triangles(program,uniform,temp,frameBuffer);
        }
        // Buffer for exchanging data between rasterizer and sdl viewer
        Matrix<uint8_t, Dynamic, Dynamic> R(width, height);
        Matrix<uint8_t, Dynamic, Dynamic> G(width, height);
        Matrix<uint8_t, Dynamic, Dynamic> B(width, height);
        Matrix<uint8_t, Dynamic, Dynamic> A(width, height);

        for (unsigned i=0; i<frameBuffer.rows();i++)
        {
            for (unsigned j=0; j<frameBuffer.cols();j++)
            {
                R(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(0);
                G(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(1);
                B(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(2);
                A(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(3);
            }
        }
        viewer.draw_image(R, G, B, A);
    };

    viewer.launch();

    return 0;
}
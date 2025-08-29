from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from threading import Lock
import numpy as np
import sys
import array
import math
import ctypes
import pyzed.sl as sl

M_PI = 3.1415926

MESH_VERTEX_SHADER = """
#version 330 core
layout(location = 0) in vec3 in_Vertex;
uniform mat4 u_mvpMatrix;
uniform vec3 u_color;
out vec3 b_color;
void main() {
    b_color = u_color;
    gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);
}
"""

FPC_VERTEX_SHADER = """
#version 330 core
layout(location = 0) in vec4 in_Vertex;
uniform mat4 u_mvpMatrix;
uniform vec3 u_color;
out vec3 b_color;
void main() {
   b_color = u_color;
   gl_Position = u_mvpMatrix * vec4(in_Vertex.xyz, 1);
}
"""

VERTEX_SHADER = """
# version 330 core
layout(location = 0) in vec3 in_Vertex;
layout(location = 1) in vec4 in_Color;
uniform mat4 u_mvpMatrix;
out vec4 b_color;
void main() {
    b_color = in_Color;
    gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);
}
"""

FRAGMENT_SHADER = """
#version 330 core
in vec3 b_color;
layout(location = 0) out vec4 color;
void main() {
   color = vec4(b_color,1);
}
"""

class Shader:
    def __init__(self, _vs, _fs):

        self.program_id = glCreateProgram()
        vertex_id = self.compile(GL_VERTEX_SHADER, _vs)
        fragment_id = self.compile(GL_FRAGMENT_SHADER, _fs)

        glAttachShader(self.program_id, vertex_id)
        glAttachShader(self.program_id, fragment_id)
        glBindAttribLocation( self.program_id, 0, "in_vertex")
        glBindAttribLocation( self.program_id, 1, "in_texCoord")
        glLinkProgram(self.program_id)

        if glGetProgramiv(self.program_id, GL_LINK_STATUS) != GL_TRUE:
            info = glGetProgramInfoLog(self.program_id)
            if (self.program_id is not None) and (self.program_id > 0) and glIsProgram(self.program_id):
                glDeleteProgram(self.program_id)
            if (vertex_id is not None) and (vertex_id > 0) and glIsShader(vertex_id):
                glDeleteShader(vertex_id)
            if (fragment_id is not None) and (fragment_id > 0) and glIsShader(fragment_id):
                glDeleteShader(fragment_id)
            raise RuntimeError('Error linking program: %s' % (info))
        if (vertex_id is not None) and (vertex_id > 0) and glIsShader(vertex_id):
            glDeleteShader(vertex_id)
        if (fragment_id is not None) and (fragment_id > 0) and glIsShader(fragment_id):
            glDeleteShader(fragment_id)

    def compile(self, _type, _src):
        try:
            shader_id = glCreateShader(_type)
            if shader_id == 0:
                print("ERROR: shader type {0} does not exist".format(_type))
                exit()

            glShaderSource(shader_id, _src)
            glCompileShader(shader_id)
            if glGetShaderiv(shader_id, GL_COMPILE_STATUS) != GL_TRUE:
                info = glGetShaderInfoLog(shader_id)
                if (shader_id is not None) and (shader_id > 0) and glIsShader(shader_id):
                    glDeleteShader(shader_id)
                raise RuntimeError('Shader compilation failed: %s' % (info))
            return shader_id
        except:
            if (shader_id is not None) and (shader_id > 0) and glIsShader(shader_id):
                glDeleteShader(shader_id)
            raise

    def get_program_id(self):
        return self.program_id

IMAGE_FRAGMENT_SHADER = """
#version 330 core
in vec2 UV;
out vec4 color;
uniform sampler2D texImage;
uniform bool revert;
uniform bool rgbflip;
void main() {
    vec2 scaler  =revert?vec2(UV.x,1.f - UV.y):vec2(UV.x,UV.y);
    vec3 rgbcolor = rgbflip?vec3(texture(texImage, scaler).zyx):vec3(texture(texImage, scaler).xyz);
    color = vec4(rgbcolor,1);
}
"""

IMAGE_VERTEX_SHADER = """
#version 330
layout(location = 0) in vec3 vert;
out vec2 UV;
void main() {
    UV = (vert.xy+vec2(1,1))/2;
    gl_Position = vec4(vert, 1);
}
"""

class ImageHandler:
    """
    Class that manages the image stream to render with OpenGL
    """
    def __init__(self):
        self.tex_id = 0
        self.image_tex = 0
        self.quad_vb = 0
        self.is_called = 0

    def close(self):
        if self.image_tex:
            self.image_tex = 0

    def initialize(self, _res):    
        self.shader_image = Shader(IMAGE_VERTEX_SHADER, IMAGE_FRAGMENT_SHADER)
        self.tex_id = glGetUniformLocation( self.shader_image.get_program_id(), "texImage")

        g_quad_vertex_buffer_data = np.array([-1, -1, 0,
                                                1, -1, 0,
                                                -1, 1, 0,
                                                -1, 1, 0,
                                                1, -1, 0,
                                                1, 1, 0], np.float32)

        self.quad_vb = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.quad_vb)
        glBufferData(GL_ARRAY_BUFFER, g_quad_vertex_buffer_data.nbytes,
                     g_quad_vertex_buffer_data, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        # Create and populate the texture
        glEnable(GL_TEXTURE_2D)

        # Generate a texture name
        self.image_tex = glGenTextures(1)
        
        # Select the created texture
        glBindTexture(GL_TEXTURE_2D, self.image_tex)
        
        # Set the texture minification and magnification filters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        
        # Fill the texture with an image
        # None means reserve texture memory, but texels are undefined
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _res.width, _res.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        
        # Unbind the texture
        glBindTexture(GL_TEXTURE_2D, 0)   

    def push_new_image(self, _zed_mat):
        glBindTexture(GL_TEXTURE_2D, self.image_tex)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, _zed_mat.get_width(), _zed_mat.get_height(), GL_RGBA, GL_UNSIGNED_BYTE,  ctypes.c_void_p(_zed_mat.get_pointer()))
        glBindTexture(GL_TEXTURE_2D, 0)            

    def draw(self):
        glUseProgram(self.shader_image.get_program_id())
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, self.image_tex)
        glUniform1i(self.tex_id, 0)

        # invert y axis and color for this image
        glUniform1i(glGetUniformLocation(self.shader_image.get_program_id(), "revert"), 1)
        glUniform1i(glGetUniformLocation(self.shader_image.get_program_id(), "rgbflip"), 1)

        glEnableVertexAttribArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, self.quad_vb)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, ctypes.c_void_p(0))
        glDrawArrays(GL_TRIANGLES, 0, 6)
        glDisableVertexAttribArray(0)
        glBindTexture(GL_TEXTURE_2D, 0)            
        glUseProgram(0)

class GLViewer:
    """
    Class that manages the rendering in OpenGL
    """
    def __init__(self):
        self.available = False
        self.mutex = Lock()
        self.draw_mesh = False
        self.new_chunks = False
        self.chunks_pushed = False
        self.change_state = False
        self.projection = sl.Matrix4f()
        self.projection.set_identity()
        self.znear = 0.5
        self.zfar = 100.
        self.image_handler = ImageHandler()
        self.sub_maps = []
        self.pose = sl.Transform().set_identity()
        self.tracking_state = sl.POSITIONAL_TRACKING_STATE.OFF
        self.mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED

    def init(self, _params, _mesh, _create_mesh): 
        glutInit()
        wnd_w = glutGet(GLUT_SCREEN_WIDTH)
        wnd_h = glutGet(GLUT_SCREEN_HEIGHT)
        width = wnd_w*0.9
        height = wnd_h*0.9
     
        if width > _params.image_size.width and height > _params.image_size.height:
            width = _params.image_size.width
            height = _params.image_size.height

        glutInitWindowSize(int(width), int(height))
        glutInitWindowPosition(0, 0) # The window opens at the upper left corner of the screen
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_SRGB)
        glutCreateWindow(b"ZED Spatial Mapping")
        glViewport(0, 0, int(width), int(height))

        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,
                      GLUT_ACTION_CONTINUE_EXECUTION)

        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)

        # Initialize image renderer
        self.image_handler.initialize(_params.image_size)

        self.init_mesh(_mesh, _create_mesh)

        # Compile and create the shader 
        if(self.draw_mesh):
            self.shader_image = Shader(MESH_VERTEX_SHADER, FRAGMENT_SHADER)
        else:
            self.shader_image = Shader(FPC_VERTEX_SHADER, FRAGMENT_SHADER)

        self.shader_MVP = glGetUniformLocation(self.shader_image.get_program_id(), "u_mvpMatrix")
        self.shader_color_loc = glGetUniformLocation(self.shader_image.get_program_id(), "u_color")
        # Create the rendering camera
        self.set_render_camera_projection(_params)

        glLineWidth(1.)
        glPointSize(4.)

        # Register the drawing function with GLUT
        glutDisplayFunc(self.draw_callback)
        # Register the function called when nothing happens
        glutIdleFunc(self.idle)   

        glutKeyboardUpFunc(self.keyReleasedCallback)

        # Register the closing function
        glutCloseFunc(self.close_func)

        self.ask_clear = False
        self.available = True

        # Set color for wireframe
        self.vertices_color = [0.12,0.53,0.84] 
        
        # Ready to start
        self.chunks_pushed = True

    def init_mesh(self, _mesh, _create_mesh):
        self.draw_mesh = _create_mesh
        self.mesh = _mesh

    def set_render_camera_projection(self, _params):
        # Just slightly move up the ZED camera FOV to make a small black border
        fov_y = (_params.v_fov + 0.5) * M_PI / 180
        fov_x = (_params.h_fov + 0.5) * M_PI / 180

        self.projection[(0,0)] = 1. / math.tan(fov_x * .5)
        self.projection[(1,1)] = 1. / math.tan(fov_y * .5)
        self.projection[(2,2)] = -(self.zfar + self.znear) / (self.zfar - self.znear)
        self.projection[(3,2)] = -1.
        self.projection[(2,3)] = -(2. * self.zfar * self.znear) / (self.zfar - self.znear)
        self.projection[(3,3)] = 0.
    
    def print_GL(self, _x, _y, _string):
        glRasterPos(_x, _y)
        for i in range(len(_string)):
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ctypes.c_int(ord(_string[i])))

    def is_available(self):
        if self.available:
            glutMainLoopEvent()
        return self.available

    def render_object(self, _object_data):      # _object_data of type sl.ObjectData
        if _object_data.tracking_state == sl.OBJECT_TRACKING_STATE.OK or _object_data.tracking_state == sl.OBJECT_TRACKING_STATE.OFF:
            return True
        else:
            return False

    def update_chunks(self):
        self.new_chunks = True
        self.chunks_pushed = False
    
    def chunks_updated(self):
        return self.chunks_pushed

    def clear_current_mesh(self):
        self.ask_clear = True
        self.new_chunks = True

    def update_view(self, _image, _pose, _tracking_state, _mapping_state):     
        self.mutex.acquire()
        if self.available:
            # update image
            self.image_handler.push_new_image(_image)
            self.pose = _pose
            self.tracking_state = _tracking_state
            self.mapping_state = _mapping_state
        self.mutex.release()
        copy_state = self.change_state
        self.change_state = False
        return copy_state

    def idle(self):
        if self.available:
            glutPostRedisplay()

    def exit(self):      
        if self.available:
            self.available = False
            self.image_handler.close()

    def close_func(self): 
        if self.available:
            self.available = False
            self.image_handler.close()      

    def keyReleasedCallback(self, key, x, y):
        if ord(key) == 113 or ord(key) == 27:   # 'q' key
            self.close_func()
        if  ord(key) == 32:                     # space bar
            self.change_state = True

    def draw_callback(self):
        if self.available:
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glClearColor(0, 0, 0, 1.0)

            self.mutex.acquire()
            self.update()
            self.draw()
            self.print_text()
            self.mutex.release()  

            glutSwapBuffers()
            glutPostRedisplay()

    # Update both Mesh and FPC
    def update(self):
        if self.new_chunks:
            if self.ask_clear:
                self.sub_maps = []
                self.ask_clear = False
            
            nb_c = len(self.mesh.chunks)

            if nb_c > len(self.sub_maps): 
                for n in range(len(self.sub_maps),nb_c):
                    self.sub_maps.append(SubMapObj())
            
            # For both Mesh and FPC
            for m in range(len(self.sub_maps)):
                if (m < nb_c) and self.mesh.chunks[m].has_been_updated:
                    if self.draw_mesh:
                        self.sub_maps[m].update_mesh(self.mesh.chunks[m])
                    else:
                        self.sub_maps[m].update_fpc(self.mesh.chunks[m])
                        
            self.new_chunks = False
            self.chunks_pushed = True

    def draw(self):  
        if self.available:
            self.image_handler.draw()

            # If the Positional tracking is good, we can draw the mesh over the current image
            if self.tracking_state == sl.POSITIONAL_TRACKING_STATE.OK and len(self.sub_maps) > 0:
                # Draw the mesh in GL_TRIANGLES with a polygon mode in line (wire)
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)

                # Send the projection and the Pose to the GLSL shader to make the projection of the 2D image
                tmp = self.pose
                tmp.inverse()
                proj = (self.projection * tmp).m
                vpMat = proj.flatten()
                
                glUseProgram(self.shader_image.get_program_id())
                glUniformMatrix4fv(self.shader_MVP, 1, GL_TRUE, (GLfloat * len(vpMat))(*vpMat))
                glUniform3fv(self.shader_color_loc, 1, (GLfloat * len(self.vertices_color))(*self.vertices_color))
        
                for m in range(len(self.sub_maps)):
                    self.sub_maps[m].draw(self.draw_mesh)

                glUseProgram(0)
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

    def print_text(self):
        if self.available:
            # Display keyboard action
            if self.mapping_state == sl.SPATIAL_MAPPING_STATE.NOT_ENABLED:
                glColor3f(0.15, 0.15, 0.15)
                self.print_GL(-0.99, 0.9, "Hit Space Bar to activate Spatial Mapping.")
            else:
                glColor3f(0.25, 0.25, 0.25)
                self.print_GL(-0.99, 0.9, "Hit Space Bar to stop spatial mapping.")

            positional_tracking_state_str = "POSITIONAL TRACKING STATE : "
            spatial_mapping_state_str = "SPATIAL MAPPING STATE : "
            state_str = ""

            # Display spatial mapping state
            if self.tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                if self.mapping_state == sl.SPATIAL_MAPPING_STATE.OK or self.mapping_state == sl.SPATIAL_MAPPING_STATE.INITIALIZING:
                    glColor3f(0.25, 0.99, 0.25)
                elif self.mapping_state == sl.SPATIAL_MAPPING_STATE.NOT_ENABLED:
                    glColor3f(0.55, 0.65, 0.55)
                else:
                    glColor3f(0.95, 0.25, 0.25)
                state_str = spatial_mapping_state_str + str(self.mapping_state)
            else:
                if self.mapping_state != sl.SPATIAL_MAPPING_STATE.NOT_ENABLED:
                    glColor3f(0.95, 0.25, 0.25)
                    state_str = positional_tracking_state_str + str(self.tracking_state)
                else:
                    glColor3f(0.55, 0.65, 0.55)
                    state_str = spatial_mapping_state_str + str(sl.SPATIAL_MAPPING_STATE.NOT_ENABLED)
            self.print_GL(-0.99, 0.83, state_str)

class SubMapObj:
    def __init__(self):
        self.current_fc = 0
        self.vboID = None
        self.index = []         # For FPC only
        self.vert = []
        self.tri = []

    def update_mesh(self, _chunk): 
        if(self.vboID is None):
            self.vboID = glGenBuffers(2)

        if len(_chunk.vertices):
            self.vert = _chunk.vertices.flatten()      # transform _chunk.vertices into 1D array 
            glBindBuffer(GL_ARRAY_BUFFER, self.vboID[0])
            glBufferData(GL_ARRAY_BUFFER, len(self.vert) * self.vert.itemsize, (GLfloat * len(self.vert))(*self.vert), GL_DYNAMIC_DRAW)
        
        if len(_chunk.triangles):
            self.tri = _chunk.triangles.flatten()      # transform _chunk.triangles into 1D array 
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.vboID[1])
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, len(self.tri) * self.tri.itemsize , (GLuint * len(self.tri))(*self.tri), GL_DYNAMIC_DRAW)
            self.current_fc = len(self.tri)

    def update_fpc(self, _chunk): 
        if(self.vboID is None):
            self.vboID = glGenBuffers(2)

        if len(_chunk.vertices):
            self.vert = _chunk.vertices.flatten()      # transform _chunk.vertices into 1D array 
            glBindBuffer(GL_ARRAY_BUFFER, self.vboID[0])
            glBufferData(GL_ARRAY_BUFFER, len(self.vert) * self.vert.itemsize, (GLfloat * len(self.vert))(*self.vert), GL_DYNAMIC_DRAW)

            for i in range(len(_chunk.vertices)):
                self.index.append(i)
            
            index_np = np.array(self.index)
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.vboID[1])
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, len(index_np) * index_np.itemsize, (GLuint * len(index_np))(*index_np), GL_DYNAMIC_DRAW)
            self.current_fc = len(index_np)

    def draw(self, _draw_mesh): 
        if self.current_fc:
            glEnableVertexAttribArray(0)
            glBindBuffer(GL_ARRAY_BUFFER, self.vboID[0])
            if _draw_mesh == True:
                glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,None)
            else:
                glVertexAttribPointer(0,4,GL_FLOAT,GL_FALSE,0,None)

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.vboID[1])
            if len(self.index) > 0:
                glDrawElements(GL_POINTS, self.current_fc, GL_UNSIGNED_INT, None)      
            else:
                glDrawElements(GL_TRIANGLES, self.current_fc, GL_UNSIGNED_INT, None)      

            glDisableVertexAttribArray(0)

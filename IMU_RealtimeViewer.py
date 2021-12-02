from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
# import serial
import pymysql

con = pymysql.connect(host="AWS퍼블릭 주소", user="Mysql 사용자", password="비밀번호",
                       db='DB이름')

sql='select imu_x,imu_y,imu_z from test order by timestamp desc limit 1;'#마지막 데이터 보여주기
# y_sql='select imu_y from test order by timestamp desc limit 1;'
# z_sql='select imu_z from test order by timestamp desc limit 1;'

cur = con.cursor()
# cur.execute(x_sql)
# print(cur.fetchall())
# ser = serial.Serial('/dev/tty.usbmodem142203', 115200, timeout=1)

ax = ay = az = 0.0
yaw_mode = True

def resize(width, height):
    if height==0:
        height=1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def drawText(position, textString):     
    font = pygame.font.SysFont ("Courier", 18, True)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))     
    textData = pygame.image.tostring(textSurface, "RGBA", True)     
    glRasterPos3d(*position)     
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def draw():
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
    
    glLoadIdentity()
    glTranslatef(-1.0,-2.0,-9.0)#-7.0

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax))

    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))
    else:
        osd_line = osd_text

    drawText((-2,-0.2, 3.5), osd_line)

    # the way I'm holding the IMU board, X and Y axis are switched 
    # with respect to the OpenGL coordinate system
    if yaw_mode:                             # experimental
        glRotatef(az, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(ay ,1.0,0.0,0.0)        # Pitch, rotate around x-axis
    glRotatef(-1*ax ,0.0,0.0,1.0)     # Roll,  rotate around z-axis

    glBegin(GL_QUADS)	
    glColor3f(0.0,1.0,0.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f(-1.0, 0.2, 1.0)		
    glVertex3f( 1.0, 0.2, 1.0)		

    glColor3f(1.0,0.5,0.0)	
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)		
    glVertex3f(-1.0,-0.2,-1.0)		
    glVertex3f( 1.0,-0.2,-1.0)		

    glColor3f(1.0,0.0,0.0)		
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)		
    glVertex3f(-1.0,-0.2, 1.0)		
    glVertex3f( 1.0,-0.2, 1.0)		

    glColor3f(1.0,1.0,0.0)	
    glVertex3f( 1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f( 1.0, 0.2,-1.0)		

    glColor3f(0.0,0.0,1.0)	
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f(-1.0,-0.2,-1.0)		
    glVertex3f(-1.0,-0.2, 1.0)		

    glColor3f(1.0,0.0,1.0)	
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)		
    glVertex3f( 1.0,-0.2,-1.0)		
    glEnd()	
         
def read_data():
    global ax, ay, az
    ax = ay = az = 0.0
    line_done = 0
    
    # cur.execute(x_sql)
    # request data by sending a dot
    # ser.write(b".") #* encode string to bytes
    #while not line_done:
    # line = ser.readline() 
    # angles = line.split(b" ")
    angles='777'
    # print(line)
    if len(angles) == 3:    
        cur = con.cursor()
        cur.execute(sql)
        datas=cur.fetchall()
        con.commit()
        # print(datas[0][0])
        ax=datas[0][0]
        ay=datas[0][1]
        az=datas[0][2]
        # print(cur.execute(x_sql))
        # ax =9
        # float(angles[0])#데이터 x넣어 주기
        # ay = cur.execute(y_sql)
        # print(cur.execute(y_sql))
        # float(angles[1])#데이터 y넣어 주기
        # az = cur.execute(z_sql)
        # print(cur.execute(z_sql))
        # float(angles[2])#데이터 z넣어 주기
        line_done = 1 

def main():
    global yaw_mode

    video_flags = OPENGL|DOUBLEBUF
    
    pygame.init()
    screen = pygame.display.set_mode((420,180), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640,480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  #* quit pygame properly
            break       
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            # ser.write(b"z")    
        read_data()
        draw()
      
        pygame.display.flip()
        frames = frames+1

    print ("fps:  %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    # ser.close()

if __name__ == '__main__': main()

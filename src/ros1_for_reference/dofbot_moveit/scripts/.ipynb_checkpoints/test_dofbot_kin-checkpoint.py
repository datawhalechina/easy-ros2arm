from math import *
import numpy
class Aubo_kinematics():
    def __init__(self):
        self.a3 =  -0.08285
        self.a4 =  -0.08285
        self.a5 =  -0.07385
        self.d1 =  0.066
        self.d2 =  0.04145
        self.ZERO_THRESH = 1e-4
    def degree_to_rad(self,q):
        temp=[]
        for i in range(len(q)):
            temp.append(q[i]*pi/180)
        return temp
    def antiSinCos(self,sA,cA):
        eps = 1e-8
        angle = 0
        if((abs(sA) < eps)and(abs(cA) < eps)):
            return 0
        if(abs(cA) < eps):
            angle = pi/2.0*self.SIGN(sA)
        elif(abs(sA) < eps):
            if (self.SIGN(cA) == 1):
                angle = 0
            else:
                angle = pi
        else:
            angle = atan2(sA, cA)
        return angle
    def SIGN(self,x):
        return (x > 0) - (x < 0)
    def aubo_forward(self,q):
        q=self.degree_to_rad(q)
        T=[]
        for i in range(16):
            T.append(0)
        print (q)
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        q4 = q[3]
        q5 = q[4]
        C3 = cos(q3)
        C4 = cos(q4)
        S3 = sin(q3)
        S4 = sin(q4)
        C12 = cos(q1+q2)
        C45 = cos(q4+q5)
        S45 = sin(q4+q5)
        S12 = sin(q1+q2)
        C345 = cos(q3 + q4 + q5)
        S345 = sin(q3 + q4 + q5)

        T[0] = C12
        T[1] = -S12*C345
        T[2] = S12*S345
        T[3] = -S12*(S3*(self.a5*C45+self.a4*C4)+C3*(self.a5*S45+self.a4*S4))-self.a3*S12*S3
        T[4] = S12
        T[5] = C12*C345
        T[6] = -C12*S345
        T[7] = C12*(S3*(self.a5*C45+self.a4*C4)+C3*(self.a5*S45+self.a4*S4))+self.a3*C12*S3
        T[8] = 0
        T[9] = S345
        T[10] = C345
        T[11] = -C3*(self.a5*C45+self.a4*C4)+S3*(self.a5*S45+self.a4*S4)+self.d1+self.d2-self.a3*C3
        T[12]=0
        T[13]=0
        T[14]=0
        T[15]=1
        return T
    def aubo_inverse(self,T):
        q_reslut_dic={}
        q_reslut=[]
        singularity = False
        num_sols = 0
        nx = T[0]
        ox = T[1]
        ax = T[2]
        px = T[3]
        ny = T[4]
        oy = T[5]
        ay = T[6]
        py = T[7]
        nz = T[8]
        oz = T[9]
        az = T[10]
        pz = T[11]
        #  shoulder rotate joint (q1) //
        q1=[0,0]
        A1 = self.d6 * ay - py
        B1 = self.d6 * ax - px
        R1 = A1 * A1 + B1 * B1 - self.d2 * self.d2
        if R1 < 0.0:
            return num_sols
        else:
            R12 = sqrt(R1)
            q1[0] =  self.antiSinCos(A1, B1) -  self.antiSinCos(self.d2, R12)
            q1[1] =  self.antiSinCos(A1, B1) -  self.antiSinCos(self.d2, -R12)
            for i in range(len(q1)):
                while q1[i] > pi:
                    q1[i] -= 2 * pi
                while q1[i] < -pi:
                    q1[i] += 2 * pi
        #// wrist 2 joint (q5) //
        q5=[[0,0],[0,0]]
        for i in range(len(q5)):
            C1 = cos(q1[i])
            S1 = sin(q1[i])
            B5 = -ay * C1 + ax * S1
            M5 = (-ny * C1 + nx * S1)
            N5 = (-oy * C1 + ox * S1)
            R5 = sqrt(M5 * M5 + N5 * N5)
            q5[i][0] = self.antiSinCos(R5, B5)
            q5[i][1] = self.antiSinCos(-R5, B5)
        #// wrist 3 joint (q6) //
        q6=0
        q3=[0,0]
        q2=[0,0]
        q4=[0,0]
        for i in range(len(q3)):
            for j in range(len(q3)):
                #// wrist 3 joint (q6) //
                C1 = cos(q1[i])
                S1 = sin(q1[i])
                S5 = sin(q5[i][j])
                A6 = (-oy * C1 + ox * S1)
                B6 = (ny * C1 - nx * S1)
                if fabs(S5) < self.ZERO_THRESH:# //the condition is only dependent on q1
                    singularity = True
                    break
                else:
                    q6 = self.antiSinCos(A6 * S5, B6 * S5)
                #/// joints (q3,q2,q4) //
                C6 = cos(q6)
                S6 = sin(q6)
                pp1 = C1 * (ax * self.d6 - px + self.d5 * ox * C6 + self.d5 * nx * S6) + S1 * (ay * self.d6 - py + self.d5 * oy * C6 + self.d5 * ny * S6)
                pp2 = -self.d1 - az * self.d6 + pz - self.d5 * oz * C6 - self.d5 * nz * S6
                B3 = (pp1 * pp1 + pp2 * pp2 - self.a2 * self.a2 - self.a3 * self.a3) / (2 * self.a2 * self.a3)
                if((1 - B3 * B3) < self.ZERO_THRESH):
                    singularity = True
                    continue
                else:
                    Sin3 = sqrt(1 - B3 * B3)
                    q3[0] = self.antiSinCos(Sin3, B3)
                    q3[1] = self.antiSinCos(-Sin3, B3)
                for k in range(len(q3)):
                    C3 = cos(q3[k])
                    S3 = sin(q3[k])
                    A2 = pp1 * (self.a2 + self.a3 * C3) + pp2 * (self.a3 * S3)
                    B2 = pp2 * (self.a2 + self.a3 * C3) - pp1 * (self.a3 * S3)
                    q2[k] = self.antiSinCos(A2, B2)
                    C2 = cos(q2[k])
                    S2 = sin(q2[k])
                    A4 = -C1 * (ox * C6 + nx * S6) - S1 * (oy * C6 + ny * S6)
                    B4 = oz * C6 + nz * S6
                    A41 = pp1 - self.a2 * S2
                    B41 = pp2 - self.a2 * C2
                    q4[k] = self.antiSinCos(A4, B4) - self.antiSinCos(A41, B41)
                    while(q4[k] > pi):
                        q4[k] -= 2 * pi
                    while(q4[k] < -pi):
                        q4[k] += 2 * pi
                    q_reslut=[q1[i],q2[k],q3[k],q4[k],q5[i][j],q6]
                    # q_sols(0,num_sols) = q1(i)
                    # q_sols(1,num_sols) = q2(k)
                    # q_sols(2,num_sols) = q3(k)
                    # q_sols(3,num_sols) = q4(k)
                    # q_sols(4,num_sols) = q5(i,j)
                    # q_sols(5,num_sols) = q6
                    q_reslut_dic.update({num_sols:q_reslut})
                    num_sols+=1
        return q_reslut_dic#,num_sols


if __name__=="__main__":
    ak47=Aubo_kinematics()
    print (ak47.aubo_forward([90,130,0,0,90]))
    # print numpy.matrix(ak47.aubo_forward([-3.3364,12.406,-81.09,-91.207,-86.08,0.164])).reshape((4,4))
    #tt=[0.010016939985065143, -0.039901099098502056, -0.9991534232559417, -0.3, -0.999934201568705, 0.005186605233011846, -0.010231894219208601, -0.09507448660946277, 0.005590478198847001, 0.999190172798396, -0.039846519755429126, 0.5962177031402299, 0, 0, 0, 1]
    # tt=[1.0, 0.0, 0.0, -0.4, 0.0, -1.0, -0.0, -0.8500000000000001, 0.0, 0.0, -1.0, -0.4, 0.0, 0.0, 0.0, 1.0]
    # q_dict,num=ak47.aubo_inverse(tt)
    # print q_dict,num
    # for i in range(len(q_dict)):
    #     print i,q_dict[i]

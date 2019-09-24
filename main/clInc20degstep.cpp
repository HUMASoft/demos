

#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "math.h"

#include "SerialArduino.h"


#include "fcontrol.h"
#include "IPlot.h"
#include "OnlineSystemIdentification.h"

#include "Kinematics.h"



int main ()
{
    //--sensor--
    SerialArduino tilt;
    float incSensor,oriSensor;
//    sleep(4); //wait for sensor

    ofstream data("/home/humasoft/code/graficas/graficas_demos/data.csv",std::ofstream::out); // /home/humasoft/code/graficas

    //Samplinfg time
    double dts=0.02;
    SamplingTime Ts(dts);


    //tau = 0.1
//    0.09516
//   ----------
//   z - 0.9048
    SystemBlock filter(0.09516,0,- 0.9048,1);

    int numOrder=1,denOrder=2;
    OnlineSystemIdentification model(numOrder, denOrder, filter, 0.98, 0.8 );
    SystemBlock sys;
    FPDBlock con(1,1,1,dts);
    PIDBlock intcon(0.1,0.05,0,dts);
    double phi,mag,w=1;



    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
//    m1.SetupPositionMode(5);
    m1.Setup_Velocity_Mode(5);


    //m2
    SocketCanPort pm2("can1");
    CiA402SetupData sd32(2048,24,0.001, 0.144);
    CiA402Device m2 (32, &pm2, &sd32);
    m2.Reset();
    m2.SwitchOn();
//    m2.SetupPositionMode(5);
    m2.Setup_Velocity_Mode(5);

    //m3
    SocketCanPort pm3("can1");
    CiA402SetupData sd33(2048,24,0.001, 0.144);
    CiA402Device m3 (33, &pm3, &sd33);
    m3.Reset();
    m3.SwitchOn();
//    m3.SetupPositionMode(5);
    m3.Setup_Velocity_Mode(5);



    //controllers
//    PIDBlock c1(2,1,0,dts);
    FPDBlock c1(8.67,20.53,-0.83,dts);
    FPDBlock c2(8.67,20.53,-0.83,dts);
    FPDBlock c3(8.67,20.53,-0.83,dts);



    double ep1,ev1,cs1;
    double tp1,tv1,v1,p1;
    double ep2,ev2,cs2;
    double tp2,tv2,v2,p2;
    double ep3,ev3,cs3;
    double tp3,tv3,v3,p3;

    double ierror, ics, cs;



    IPlot plot1,plot2,plot3,id;


    //--Neck Kinematics--
    double l0=0.1085;
    double lg0=l0+0.002;
    double radius=0.0075; //winch radius
    GeoInkinematics neck_ik(0.052,0.052,l0); //kinematics geometric
    vector<double> lengths(3);

    double inc=20.0; //inclination tendon length
    double incVel;
    double ori=0*M_PI/180; //target orientation
    double da2=2*M_PI/3, da3=4*M_PI/3; //angle shift for tendons 2 and 3

    //tilt initialization
    for (double t=0; t<6; t+=dts)
    {
    if (tilt.readSensor(incSensor,oriSensor)>=0) break;

    }

//    neck_ik.GetIK(inc,90,lengths);
//    tp1=(lg0-lengths[0])/radio;
//    tp2=(lg0-lengths[1])/radio;
//    tp3=(lg0-lengths[2])/radio;

//    cout << "tp1 " << tp1 << ", tp2 " << tp2 << ", tp3 " << tp3 <<endl;

//    vector<double> inc(interval/dts);
//    for i


    //TODO go and back to 0 reps times.
    //TODO check why negative values break the control
    for (long loops = 1; loops > 0 ; loops--)
    {
        cout << "Going to: " << inc << endl;

        double interval=5; //in seconds
        for (double t=0;t<interval; t+=dts)
        {
            if (tilt.readSensor(incSensor,oriSensor) <0)
            {
                cout << "Sensor error! " << endl;
                //Due to sensor error set motors zero velocity.
                m1.SetVelocity(0);
                m2.SetVelocity(0);
                m3.SetVelocity(0);

            }


            //negative feedback
            ierror = inc - incSensor;
            cout << "ierror " <<  ierror  << ", cs " << cs << ", incSensor " << incSensor <<endl;
            data << t << " , " <<ierror<< " , "<< incSensor <<endl;

            //velocity strategy (activate also SetupVelocityMode())


            ierror= ierror*M_PI/180; //degrees/sec to rad/sec
            //controller computes control signal
            //        cs = ierror > con;
            cs = ierror > intcon;

            if (!isnormal(cs)) cs = 0;


            cs1=(cs*cos(ori))/radius;
            cs2=(cs*cos(ori+da2))/radius;
            cs3=(cs*cos(ori+da3))/radius;
            m1.SetVelocity(cs1);
            m2.SetVelocity(cs2);
            m3.SetVelocity(cs3);
            //        cout << "cs1 " << cs1 << ", cs2 " << cs2 << ", cs3 " << cs3 <<endl;


            //velocity strategy (activate also SetupVelocityMode())




            //Position Strategy (activate also SetupPositionMode())
            /*
        //controlled inclination (cs)
        cs = inc + (ierror > con);
        neck_ik.GetIK(cs,ori*180/M_PI,lengths);
        tp1=(lg0-lengths[0])/radius;
        tp2=(lg0-lengths[1])/radius;
        tp3=(lg0-lengths[2])/radius;
        m1.SetPosition(tp1);
        m2.SetPosition(tp2);
        m3.SetPosition(tp3);

//        cout << "tp1 " << tp1 << ", tp2 " << tp2 << ", tp3 " << tp3 <<endl;
*/
            //Position Strategy (activate also SetupPositionMode())

            /* Velocity local loops
        file << t << ",";

        //loopm1
        ep1=tp1- m1.GetPosition();
        cs1= ep1 > c1;
        m1.SetVelocity(cs1);
        p1=m1.GetPosition();
        plot1.pushBack(p1);

        file << tp1 << "," << p1 << ","<< cs1 << ",";

        //loopm2
        ep2=tp2- m2.GetPosition();
        cs2= ep2 > c2;
        m2.SetVelocity(cs2);
        p2=m2.GetPosition();
        plot2.pushBack(p2);

        file << tp2 << ","<< p2 << ","<< cs2 << ",";


        //loopm3
        ep3=tp3- m3.GetPosition();
        cs3= ep3 > c3;
        m3.SetVelocity(cs3);
        p3=m3.GetPosition();
        plot3.pushBack(p3);

        file << tp3 << ","<< p3 << ","<< cs3  << endl;

*/

            Ts.WaitSamplingTime();
        }

    }



    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);

    data.close();



return 0;

}


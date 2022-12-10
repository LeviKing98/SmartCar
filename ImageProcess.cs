// 2020.8.13
// by LeviKing98
// github: https://github.com/LeviKing98

// 图像大小：70 * 186
// 2020年智能车竞赛 双车组/三轮车 图像处理代码 (C#)
#define UpperComputer
using System;
using System.Collections.Generic;

namespace MyNrf.这里写仿真程序
{

    #region 透视变换结构体
    class point
    {
        public int x;
        public int y;
    }

    class Trans_Marix
    {
        public float x11;
        public float x12;
        public float x13;
        public float x21;
        public float x22;
        public float x23;
        public float x31;
        public float x32;
        public float x33;
    }
    #endregion
    #region 模糊控制结构体

    class FuzPID
    {
        public float Maximum = 300;
        public float Minimum = -300;

        public float Proportion = 10;
        public float MaxProportion = 25;
        public float MinProportion = 10;
        public float Proportion_K = 0.8f;

        public float Integral = 0;
        public float MaxIntegral = 5;
        public float MinIntegral = -5;
        public float Integral_K = 0;

        public float Derivative = 10;
        public float MaxDerivative = 20;
        public float MinDerivative = 5;
        public float Derivative_K = 3.0f;

        public float E = 0;
        public byte[] indexE = new byte[2] { 0, 0 };
        public float[] MsE = new float[2] { 0f, 0f };

        public float EC = 0;
        public byte[] indexEC = new byte[2] { 0, 0 };
        public float[] MsEC = new float[2] { 0f, 0f };

        public float LastError = 0;
        public float PrevError = 0;
        public float SumError = 0;
        public long Output = 0;

    }
    #endregion
    class SmartProcess
    {
        #region 横向扫点记录
        byte[] L_black = new byte[70];
        byte[] R_black = new byte[70];
        byte[] L_Start = new byte[70];
        byte[] Tang_Line = new byte[70];
        byte[] L_Trans = new byte[70];
        byte[] R_Trans = new byte[70];
        byte[] LCenter = new byte[70];
        public byte[,] Mask1 = new byte[70, 186];
        public byte[,] Mask2 = new byte[70, 186];
        #endregion
        #region 系统参数及按键函数
        byte black = 0, white = 255;
        byte x, y;
        byte OlLine, OlRow;
        private Form1 mf;
        byte[][] J_Pixels = new byte[70][]
        {
            new byte[186], new byte[186],  new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186],
            new byte[186], new byte[186],  new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186],
            new byte[186], new byte[186],  new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186],
            new byte[186], new byte[186],  new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186],
            new byte[186], new byte[186],  new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186],
            new byte[186], new byte[186],  new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186],
            new byte[186], new byte[186],  new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186], new byte[186],
        };
        public void RoadInit()
        {
            RoadType = RoadTypeEnum.Common;
            Circle_State = Circle.Circle_0;
            Garage_State = Garage.Garage_0;
            Meeting_State = Meeting_st.Meeting_0;
            Slope_State = Slope_st.Slope_0;
            Out_Garage_State = Out_Garage.Out_Garage_0;
            Circle_cnt = 0;
            MeetingFlag = 0;
        }
        #endregion 
        #region 传感器数据
        // 陀螺仪加速度计数据
        int AngleZ;
        int GyrpY;
        int GyrpX;
        int Encoder_Cnt, Encoder_L_Cnt, Encoder_R_Cnt; // 编码器数据
        #endregion
        #region 小车状态解算函数及变量
        float Actual_Speed, Last_Speed, Actual_R_Speed, Actual_L_Speed;
        float Actual_AnguarSpeed;
        float Acc;
        float dt = 0.02f;
        /* void State_Cale()
         {
 #if UpperComputer
             Last_Speed = Actual_Speed;

             //小齿轮数为 30 
             //大齿轮数为 68
             //编码器读取时间间隔为 0.02s
             //编码器一圈度数为 1024
             //轮胎半径为 0.064 M
             //k = (1/1024)*(30/68)*(3.14*0.064)/0.02 = 0.00433

             Actual_Speed = Encoder_Cnt * 0.00433f;
             Actual_L_Speed = Encoder_L_Cnt * 0.00433f;
             Actual_R_Speed = Encoder_R_Cnt * 0.00433f;

             //默认配置为 每单位数字量 代表 62.5 mdps 毫度每秒
             //转换为 弧度 速度 
             //k = (n*62.5/1000)*(3.14/180) = 0.00109

             Actual_AnguarSpeed = GyrpY * 0.00109f;
             //线加速度计算
             Acc = (Actual_Speed - Last_Speed) / dt;
             SetText_2("Actual_Speed = " + Actual_Speed);
             //SetText_2("Acc = " + Acc);
             SetText_2("Actual_AnguarSpeed = " + Actual_AnguarSpeed);
             SetText_2("AngleZ = " + AngleZ);
             //SetText_2("AngleZ_Correction = " + AngleZ_Correction);
             SetText_2("Trans_k = " + Trans_k);
 #else
             Last_Speed = Actual_Speed;

             //小齿轮数为 30 
             //大齿轮数为 68
             //编码器读取时间间隔为 0.02s
             //编码器一圈度数为 1024
             //轮胎半径为 0.064 M
             //k = (n/1024)*(30/68)*(3.14*0.064)/0.02 = 0.00433

             Actual_Speed = Encoder_Cnt * 0.00433f;
             Actual_L_Speed = Encoder_L_Cnt * 0.00433f;
             Actual_R_Speed = Encoder_R_Cnt * 0.00433f;

             //默认配置为 每单位数字量 代表 62.5 mdps 毫度每秒
             //转换为 弧度 速度 
             //k = (n*62.5/1000)*(3.14/180) = 0.00109

             Actual_AnguarSpeed = FXAS21002_AccelData.y * 0.00109f;
             //线加速度计算
             Acc = (Actual_Speed - Last_Speed) / dt;
 #endif
         }*/
        #endregion
        #region 边线拟合函数及变量
        float[,] Str_Line_kb = new float[2, 2]; // 下标1：0 为左线 ，1为右线；下标2：0为斜率，1为截距
        float MidRegk, MidRegb;
        float my_fabs(float a)
        {
            if (a > 0)
                return a;
            else
                return -a;
        }
        void Str_Regression_Left(byte startline, byte endline, byte checkFlag) // 输入左边线的起始、结束行，进行左边线的一元一次直线回归，赋给左边线的拟合直线参数
        {
            int i;
            float SumUp, SumDown, avrX, avrY, SumX = 0, SumY = 0, SumLines = 0;
            byte interval;
            int Endline = endline < 69 ? endline : 69;
            if (Endline - startline >= 20)
                interval = 4;
            else if (Endline - startline >= 10)
                interval = 2;
            else
                interval = 1;
            for (i = startline; i < Endline; i++)
            {
                if (i % interval == 0)
                {
                    if (checkFlag == 1 && my_fabs(L_black[i] - L_black[startline]) > 10)
                    {
                        SetText_1("! Wrong Reg Point L_black[" + i + "] = " + L_black[i]);
                        Endline = i - 1;
                        break;
                    }
                    SumLines++;   // startline 为开始行， //Endline 结束行 //SumLines
                    SumX += i;
                    SumY += L_black[i];
                }
            }
            avrX = SumX / SumLines;     //X的平均值
            avrY = SumY / SumLines;     //Y的平均值       
            SumUp = 0;
            SumDown = 0;
            for (i = startline; i < Endline; i++)
            {
                if (i % interval == 0)
                {
                    SumUp += ((float)L_black[i] - avrY) * (i - avrX);
                    SumDown += (i - avrX) * (i - avrX);
                }
            }
            if (SumDown == 0)
                Str_Line_kb[0, 0] = 0;
            else
                Str_Line_kb[0, 0] = (float)(SumUp / SumDown);
            Str_Line_kb[0, 1] = (SumY - Str_Line_kb[0, 0] * SumX) / (float)SumLines;  //截距
            //SetText_1("Str_Line_kb[0, 0] = "+ Str_Line_kb[0, 0]+ " Str_Line_kb[0, 1] = "+ Str_Line_kb[0, 1]);
        }
        void Str_Regression_Right(byte startline, byte endline, byte checkFlag) // 输入右边线的起始、结束行，进行右边线的一元一次直线回归，赋给右边线的拟合直线参数

        {
            int i;
            float SumUp, SumDown, avrX, avrY, SumX = 0, SumY = 0, SumLines = 0;
            byte interval;
            int Endline = endline < 69 ? endline : 69;
            if (Endline - startline >= 20)
                interval = 4;
            else if (Endline - startline >= 10)
                interval = 2;
            else
                interval = 1;
            for (i = startline; i < Endline; i++)
            {
                if (i % interval == 0)
                {
                    if (checkFlag == 1 && my_fabs(R_black[i] - R_black[startline]) > 10)
                    {
                        SetText_1("! Wrong Reg Point R_black[" + i + "] = " + R_black[i]);
                        Endline = i - 1;
                        break;
                    }
                    SumLines++;
                    SumX += i;
                    SumY += R_black[i];
                    //SetText_1("R_black["+i+"] = "+ R_black[i]);
                }
            }
            avrX = SumX / SumLines;     //X的平均值
            avrY = SumY / SumLines;     //Y的平均值       
            SumUp = 0;
            SumDown = 0;
            for (i = startline; i < Endline; i++)
            {
                if (i % interval == 0)
                {
                    SumUp += ((float)R_black[i] - avrY) * (i - avrX);
                    SumDown += (i - avrX) * (i - avrX);
                }
            }
            if (SumDown == 0)
                Str_Line_kb[1, 0] = 0;
            else
                Str_Line_kb[1, 0] = (float)(SumUp / SumDown);
            Str_Line_kb[1, 1] = (SumY - Str_Line_kb[1, 0] * SumX) / (float)SumLines;  //截距\
        }
        void Str_Regression_Mid(byte startline, byte endline) // 同上
        {
            int i;
            float SumUp, SumDown, avrX, avrY, SumX = 0, SumY = 0, SumLines = 0;
            byte interval;
            int Endline = endline < 69 ? endline : 69;
            if (Endline - startline >= 20)
                interval = 4;
            else if (Endline - startline >= 10)
                interval = 2;
            else
                interval = 1;
            for (i = startline; i < Endline; i++)
            {
                if (i % interval == 0)
                {
                    /*if (checkFlag == 1 && my_fabs(L_black[i] - L_black[startline]) > 10)
                    {
                        SetText_1("! Wrong Reg Point L_black[" + i + "] = " + L_black[i]);
                        Endline = i - 1;
                        break;
                    }*/
                    SumLines++;   // startline 为开始行， //Endline 结束行 //SumLines
                    SumX += i;
                    SumY += LCenter[i];
                    //SetText_1(" LCenter["+i+"]" + LCenter[i]);
                }
            }
            avrX = SumX / SumLines;     //X的平均值
            avrY = SumY / SumLines;     //Y的平均值       
            SumUp = 0;
            SumDown = 0;
            for (i = startline; i < Endline; i++)
            {
                if (i % interval == 0)
                {
                    SumUp += ((float)LCenter[i] - avrY) * (i - avrX);
                    SumDown += (i - avrX) * (i - avrX);
                }
            }
            if (SumDown == 0)
                MidRegk = 0;
            else
                MidRegk = (float)(SumUp / SumDown);
            MidRegb = (SumY - MidRegk * SumX) / (float)SumLines;  //截距
            //SetText_1("Str_Line_kb[0, 0] = "+ Str_Line_kb[0, 0]+ " Str_Line_kb[0, 1] = "+ Str_Line_kb[0, 1]);
        }
        void Str_LineParaCale_Left(byte x1, byte y1, byte x2, byte y2) // 输入两个点，直接计算一元一次直线参数，赋给左边线的拟合直线参数
        {
            Str_Line_kb[0, 0] = (float)(y2 - y1) / (float)(x2 - x1);
            Str_Line_kb[0, 1] = (float)y2 - Str_Line_kb[0, 0] * (float)x2;
        }
        void Str_LineParaCale_Right(byte x1, byte y1, byte x2, byte y2) // 输入两个点，直接计算一元一次直线参数，赋给右边线的拟合直线参数
        {
            Str_Line_kb[1, 0] = (float)(y2 - y1) / (float)(x2 - x1);
            Str_Line_kb[1, 1] = (float)y2 - Str_Line_kb[1, 0] * (float)x2;
        }
        byte Str_LinePointSet_Left(byte x) // 输入一个点的x坐标，计算该点在的左边线拟合直线的y坐标。x为行，y为列
        {
            float y;
            y = Str_Line_kb[0, 0] * (float)x + Str_Line_kb[0, 1];
            if (y > 185)
                return 185;
            else if (y < 0)
                return 0;
            else
                return (byte)y;
        }
        byte Str_LinePointSet_Right(byte x) // 输入一个点的x坐标，计算该点在的右边线拟合直线的y坐标。x为行，y为列
        {
            float y;
            y = Str_Line_kb[1, 0] * (float)x + Str_Line_kb[1, 1];
            if (y > 185)
                return 185;
            else if (y < 0)
                return 0;
            else
                return (byte)y;
        }
        void Str_LineSet_Left(byte x1, byte x2) // 输入两个点的x坐标，计算两点之间的左边线拟合直线。x为行，y为列
        {
            byte i;
            for (i = x1; i <= x2 && i < 70; i++)
            {
                L_black[i] = Str_LinePointSet_Left(i);
                //L_Trans[i] = Str_LinePointSet_Left(i);
            }
        }
        void Str_LineSet_Right(byte x1, byte x2) // 同上
        {
            byte i;
            for (i = x1; i <= x2 && i < 70; i++)
            {
                R_black[i] = Str_LinePointSet_Right(i);
                //SetText_1("R_black["+i+"] = "+ R_black[i]);
                //R_Trans[i] = Str_LinePointSet_Right(i);
            }
        }
        float Str_Quant(byte startRow, byte endRow, byte mod) // 计算边线的点与拟合直线的离散度。返回值越高，边线点的分与拟合直线的差距越大。
        {
            float sum = 0;
            float y;
            byte count = 0;
            byte interval = 1;
            /*if (endRow - startRow >= 20)
                interval = 4;
            else if (endRow - startRow >= 10)
                interval = 2;
            else
                interval = 1;*/
            if (mod == 0)  //左边线直线度量化
            {
                SetText_1("Left Str Quant Star ,Start Row: " + startRow + " EndRow = " + endRow);
                //Str_Regression_Left(startRow, endRow, 0);
                for (OlRow = startRow; OlRow < endRow; OlRow++)
                {
                    //SetText_1("OlRow = "+OlRow);
                    if (OlRow % interval == 0)
                    {
                        if (L_black[OlRow] != 185)
                        {
                            count++;
                            y = Str_Line_kb[0, 0] * OlRow + Str_Line_kb[0, 1];
                            if (y > 185)
                                y = 185;
                            else if (y < 0)
                                y = 0;
                            Mask1[OlRow, (int)y] = 1;
                            sum += my_fabs(L_black[OlRow] - y);
                            //SetText_1("Left Row = " + OlRow + " Error = " + my_fabs(L_black[OlRow] - y) + " sum = " + sum);
                        }
                    }
                }
                sum /= count;
                //SetText_2(startRow +" - "+ endRow + " Left StrError = "+ sum);
            }
            else if (mod == 1)
            {
                SetText_1("Right Str Quant Star ,Start Row: " + startRow + " EndRow = " + endRow);
                //Str_Regression_Right(startRow, endRow, 0);
                for (OlRow = startRow; OlRow < endRow; OlRow++)
                {
                    //SetText_1("OlRow = " + OlRow);
                    if (OlRow % interval == 0)
                    {
                        if (R_black[OlRow] != 0)
                        {
                            count++;
                            y = Str_Line_kb[1, 0] * OlRow + Str_Line_kb[1, 1];
                            if (y > 185)
                                y = 185;
                            else if (y < 0)
                                y = 0;
                            Mask1[OlRow, (int)y] = 1;
                            sum += my_fabs(R_black[OlRow] - y);
                            //SetText_1("Right Row = " + OlRow + " Error = " + my_fabs(R_black[OlRow] - y) + " sum = " + sum);
                        }
                    }
                }
                sum /= count;
                //SetText_2(startRow + " - " + endRow + " Right StrError = " + sum);
            }
            else if (mod == 2)
            {
                SetText_1("Mid Str Quant Star ,Start Row: " + startRow + " EndRow = " + endRow);
                //Str_Regression_Right(startRow, endRow, 0);
                for (OlRow = startRow; OlRow < endRow; OlRow++)
                {
                    //SetText_1("OlRow = " + OlRow);
                    if (OlRow % interval == 0)
                    {
                        //if (LCenter[OlRow] != 0)
                        //{
                        count++;
                        y = MidRegk * OlRow + MidRegb;
                        if (y > 185)
                            y = 185;
                        else if (y < 0)
                            y = 0;
                        Mask1[OlRow, (int)y] = 1;
                        sum += my_fabs(LCenter[OlRow] - y);
                        //SetText_1("Mid Row = " + OlRow + " LCenter[" + OlRow + "] = " + LCenter[OlRow] + " Cale y = "+ y + " Error = " + my_fabs(LCenter[OlRow] - y));
                        //}
                    }
                }
                sum /= count;
                //SetText_2(startRow + " - " + endRow + " Right StrError = " + sum);
            }
            SetText_1("Error = " + sum);
            return sum;
        }
        float[,] Quad_Line_ab = new float[2, 2];
        void Quad_Regression_Left(byte startline, byte endline) // 左边线的二次曲线拟合
        {
            int i;
            float SumX4 = 0, SumY = 0, SumX2 = 0, SunX2Y = 0, SumLines = 0;
            float Ave_x2, Ave_y, Ave_x4, Ave_x2y;
            byte interval;
            if (endline - startline >= 12)
                interval = 4;
            else if (endline - startline >= 6)
                interval = 2;
            else
                interval = 1;
            for (i = startline; i <= endline; i++)
            {
                if (i % interval == 0)
                {
                    SumLines++;   // startline 为开始行， //endline 结束行 //SumLines
                    SumX2 += i * i;
                    SumX4 += i * i * i * i;
                    SunX2Y += i * i * L_black[i];
                    SumY += L_black[i];
                }
            }
            Ave_x2 = SumX2 / SumLines;     //X的平均值
            Ave_x4 = SumX4 / SumLines;
            Ave_y = SumY / SumLines;     //Y的平均值       
            Ave_x2y = SunX2Y / SumLines;
            Quad_Line_ab[0, 0] = (Ave_x2y - Ave_y * Ave_x2) / (Ave_x4 - Ave_x2 * Ave_x2);
            if (Quad_Line_ab[0, 0] < -0.035f)
                Quad_Line_ab[0, 0] = -0.035f;
            Quad_Line_ab[0, 1] = -Quad_Line_ab[0, 0] * Ave_x2 + Ave_y;

            SetText_1("Quad_Line_ab[0, 0] = " + Quad_Line_ab[0, 0] + " Quad_Line_ab[0, 1] =  " + Quad_Line_ab[0, 1]);
        }
        byte Quad_LinePointSet_Left(byte x) // 给定x,得出拟合曲线的y
        {
            float y;
            y = Quad_Line_ab[0, 0] * x * x + Quad_Line_ab[0, 1];
            if (y > 185)
                return 185;
            else if (y < 0)
                return 0;
            else
                return (byte)y;
        }
        void Quad_LineSet_Left(byte x1, byte x2) // 给定x1,x2，得出拟合x1与x2之间的拟合曲线。x为行
        {
            byte i;
            for (i = x1; i <= x2; i++)
            {
                L_black[i] = Quad_LinePointSet_Left(i);
                //L_Trans[i] = Str_LinePointSet_Left(i);
            }
        }
        void Quad_Regression_Right(byte startline, byte endline) // 右边线，同上
        {
            int i;
            float SumX4 = 0, SumY = 0, SumX2 = 0, SunX2Y = 0, SumLines = 0;
            float Ave_x2, Ave_y, Ave_x4, Ave_x2y;
            byte interval;
            if (endline - startline >= 12)
                interval = 4;
            else if (endline - startline >= 6)
                interval = 2;
            else
                interval = 1;
            for (i = startline; i <= endline; i++)
            {
                if (i % interval == 0)
                {
                    SumLines++;   // startline 为开始行， //endline 结束行 //SumLines
                    SumX2 += i * i;
                    SumX4 += i * i * i * i;
                    SunX2Y += i * i * R_black[i];
                    SumY += R_black[i];
                    //SetText_1("R_black["+i+"] = "+ R_black[i]);
                }
            }
            Ave_x2 = SumX2 / SumLines;     //X的平均值
            Ave_x4 = SumX4 / SumLines;
            Ave_y = SumY / SumLines;     //Y的平均值       
            Ave_x2y = SunX2Y / SumLines;
            Quad_Line_ab[1, 0] = (Ave_x2y - Ave_y * Ave_x2) / (Ave_x4 - Ave_x2 * Ave_x2);
            if (Quad_Line_ab[1, 0] > 0.035f)
                Quad_Line_ab[1, 0] = 0.035f;
            Quad_Line_ab[1, 1] = -Quad_Line_ab[1, 0] * Ave_x2 + Ave_y;
            SetText_1("Quad_Line_ab[1, 0] = " + Quad_Line_ab[1, 0] + " Quad_Line_ab[1, 1] =  " + Quad_Line_ab[1, 1]);
        }
        byte Quad_LinePointSet_Right(byte x) // 右边线，同上
        {
            float y;
            y = Quad_Line_ab[1, 0] * x * x + Quad_Line_ab[1, 1];
            if (y > 185)
                return 185;
            else if (y < 0)
                return 0;
            else
                return (byte)y;
        }
        void Quad_LineSet_Right(byte x1, byte x2) // 右边线，同上
        {
            byte i;
            for (i = x1; i <= x2; i++)
            {
                R_black[i] = Quad_LinePointSet_Right(i);
                //L_Trans[i] = Str_LinePointSet_Left(i);
            }
        }
        #endregion
        #region 八邻域扫线函数及变量
        public enum Scan_LineType { RightType, LeftType }; // 扫描类型：左右
        public enum Scan_Direction { Vertical, Horizontal }; // 种子寻找方向：横向纵向
        public enum RoadNum { RoadNum_1, RoadNum_2 }; // 路段：第一第二
        struct LineStatus//定义记录左右线特征结构体 
        {
            public byte StartLine; // 扫描开始行
            public byte EndLine; // 扫面结束行
            public byte Agl_Row; // 第一个拐点行
            public byte Agl_Line; // 第一个拐点列
            public byte Agl_PointNum; // 第一个拐点索引
            public byte Agl_2_Row;  // 第二个拐点行 
            public byte Agl_2_Line; // 第二个拐点列         
            public byte Agl_2_PointNum; // 第二个拐点索引
            public byte BroadWire_Cnt; // 左边线中，向 正左/左下 延伸的点；右边线中，向 正右/右下 延伸的点； 主要用来评估十字赛道。
            public byte Reserve_Cnt; // 左右边线中，向 正下 延伸的点
            public byte Straight_Cnt; // 左边线中，向 正上/右上 延伸的点；右边线中，向 正上/左上 延伸的点； 主要用来评估直道。
            public byte Turn_Cnt; // 左边线中，向 正右 延伸的点；右边线中，向 正左 延伸的点； 主要用来评估弯道。
            public byte PointCnt; // 八领域扫描到的总点数
            public byte Err_Cnt; // 错误点数
            public byte Error; // 扫描是否异常
            public byte Rec_Point; // 八领域扫描到的有效点数
            public float RecStrError;
            public float AglStrError;
            public float Line_k; // 拟合直线的k
            public float Line_b; // 拟合直线的b
        };
        public struct Margin
        {
            public byte row;
            public byte line;
            public byte direction;//从第二个点开始记录方向
        }//八邻域扫线
        enum NC_Direction { NC_downright, NC_equalright, NC_upright, NC_upmiddle, NC_upleft, NC_equalleft, NC_downleft, NC_downmiddle };//逆时针扫线顺序
        enum SC_Direction { SC_downleft, SC_equalleft, SC_upleft, SC_upmiddle, SC_upright, SC_equalright, SC_downright, SC_downmiddle };//顺时针扫线
        public Margin[] RightMargin_1 = new Margin[180];
        public Margin[] LeftMargin_1 = new Margin[180];
        public Margin[] RightMargin_2 = new Margin[150];
        public Margin[] LeftMargin_2 = new Margin[150];
        LineStatus RightLine_1, LeftLine_1, RightLine_2, LeftLine_2;
        public Margin[] Car_RightMargin = new Margin[50];
        public Margin[] Car_LeftMargin = new Margin[50];
        LineStatus Car_RightLine, Car_LeftLine;
        public Margin[] Zebra_Margin = new Margin[100];
        LineStatus Zebra_Line;
        byte Record_Flag;
        byte Error_Flag;
        byte InitSeedLine = 93;
        byte Reflect_Detect(Scan_LineType Line_RorL, RoadNum Road_Num, byte point) // 检测反光
        {
            if (point < 3)
                return 0;
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Road_Num == RoadNum.RoadNum_1)
                {
                    for (int x = 1; x < 3; x++)
                    {
                        if ((LeftMargin_1[point - x].direction + 4) % 8 == LeftMargin_1[point].direction && LeftMargin_1[point].row > 15 && LeftMargin_1[point].row < 60)
                        {
                            SetText_1("!" + Line_RorL + " " + Road_Num + " Error_Point " + point + " Row = " + LeftMargin_1[point].row + " line  = " + LeftMargin_1[point].line);
                            LeftLine_1.Err_Cnt++;
                            if (LeftLine_1.Err_Cnt >= 2)
                            {
                                Error_Flag = 1;
                                LeftLine_1.Error = 1;
                                LeftLine_1.Err_Cnt = point;
                                setText用户自定义("Left Line 1 Error");
                                SetText_1("! LeftLine_1.Err_Cnt = " + LeftLine_1.Err_Cnt);
                                return 1;
                            }
                            else return 0;
                        }
                    }
                }
                else
                {
                    for (int x = 1; x < 3; x++)
                    {
                        if ((LeftMargin_2[point - x].direction + 4) % 8 == LeftMargin_2[point].direction && LeftMargin_2[point].row > 15 && LeftMargin_2[point].row < 60)
                        {
                            SetText_1("!" + Line_RorL + " " + Road_Num + " Error_Point = " + point + " Row = " + LeftMargin_2[point].row + " line  = " + LeftMargin_2[point].line);
                            LeftLine_2.Err_Cnt++;
                            if (LeftLine_2.Err_Cnt >= 2)
                            {
                                Error_Flag = 1;
                                LeftLine_2.Error = 1;
                                LeftLine_2.Err_Cnt = point;
                                setText用户自定义("Left Line 2 Error");
                                SetText_1("! LeftLine_2.Err_Cnt = " + LeftLine_2.Err_Cnt);
                                return 1;
                            }
                            else return 0;
                        }
                    }
                }
            }
            else
            {
                if (Road_Num == RoadNum.RoadNum_1)
                {
                    for (int x = 1; x < 3; x++)
                    {
                        if ((RightMargin_1[point - x].direction + 4) % 8 == RightMargin_1[point].direction && RightMargin_1[point].row > 15 && RightMargin_1[point].row < 60)
                        {
                            SetText_1("!" + Line_RorL + " " + Road_Num + " Error_Point = " + point + " Row = " + RightMargin_1[point].row + " line  = " + RightMargin_1[point].line);
                            RightLine_1.Err_Cnt++;
                            if (RightLine_1.Err_Cnt >= 2)
                            {
                                Error_Flag = 1;
                                RightLine_1.Error = 1;
                                RightLine_1.Err_Cnt = point;
                                setText用户自定义("Right Line 1 Error");
                                SetText_1("! RightLine_1.Err_Cnt = " + RightLine_1.Err_Cnt);
                                return 1;
                            }
                            else return 0;
                        }
                    }
                }
                else
                {
                    for (int x = 1; x < 3; x++)
                    {
                        if ((RightMargin_2[point - x].direction + 4) % 8 == RightMargin_2[point].direction && RightMargin_2[point].row > 15 && RightMargin_2[point].row < 60)
                        {
                            SetText_1("!" + Line_RorL + " " + Road_Num + " Error_Point = " + point + " Row = " + RightMargin_2[point].row + " line  = " + RightMargin_2[point].line);
                            RightLine_2.Err_Cnt++;
                            if (RightLine_2.Err_Cnt >= 2)
                            {

                                Error_Flag = 1;
                                RightLine_2.Error = 1;
                                RightLine_2.Err_Cnt = point;
                                setText用户自定义("Right Line 2 Error");
                                SetText_1("! RightLine_2.Err_Cnt = " + RightLine_2.Err_Cnt);
                                return 1;
                            }
                            else return 0;
                        }
                    }
                }
            }
            return 0;
        }
        byte CarReflect_Detect(Scan_LineType Line_RorL, byte point) // 检测车反光
        {
            //SetText_1("Car Reflect Test");
            if (point < 3)
                return 0;
            if (Line_RorL == Scan_LineType.LeftType)
            {
                //if(Car_LeftLine.)
                for (int x = 1; x < 3; x++)
                {
                    //SetText_1("Car RefDect Car_LeftMargin[" + (point - x) + "].direction + 4) % 8 = " + (Car_LeftMargin[point - x].direction + 4 % 8) + " Car_LeftMargin[" + point + "].direction = " + Car_LeftMargin[point].direction);
                    if ((Car_LeftMargin[point - x].direction + 4) % 8 == Car_LeftMargin[point].direction)
                    {
                        SetText_1("!" + Line_RorL + " Car Error_Point " + point + " Row = " + Car_LeftMargin[point].row + " line  = " + Car_LeftMargin[point].line);
                        Car_LeftLine.Err_Cnt++;
                        if (Car_LeftLine.Err_Cnt >= 2)
                        {
                            Car_LeftLine.Error = 1;
                            setText用户自定义("Car Left Line 1 Error");
                            SetText_1("!Error");
                            return 1;
                        }
                        else return 0;
                    }
                }
                if (Car_LeftMargin[point].row == 69)
                {
                    //SetText_1("Car Left Region Wrong!");
                    //Car_LeftLine.Error = 1;
                    return 1;
                }
            }
            else
            {
                for (int x = 1; x < 3; x++)
                {
                    if ((Car_RightMargin[point - x].direction + 4) % 8 == Car_RightMargin[point].direction)
                    {
                        SetText_1("!" + Line_RorL + "Car Error_Point = " + point + " Row = " + Car_RightMargin[point].row + " line  = " + Car_RightMargin[point].line);
                        Car_RightLine.Err_Cnt++;
                        if (Car_RightLine.Err_Cnt >= 2)
                        {
                            Car_RightLine.Error = 1;
                            setText用户自定义("Car Right Line 1 Error");
                            SetText_1("!Error");
                            return 1;
                        }
                        else return 0;
                    }
                }
                if (Car_RightMargin[point].row == 69)
                {
                    //SetText_1("Car Right Region Wrong!");
                    //Car_RightLine.Error = 1;
                    return 1;
                }
            }
            return 0;
        }
        void Camera_Protect() // 摄像头保护
        {
            byte count = 0;
            byte j = 0;
            for (j = 0; j < 185; j = (byte)(j + 10))
            {
                if (J_Pixels[1][j] == black)
                    count++;
            }
            if (count == 18)
            {
                setText用户自定义("摄像头保护");
            }
        }
        void LineMargin_Init() // 八邻域初始化
        {
            //右
            RightLine_1.StartLine = 0;
            RightLine_1.EndLine = 0;
            RightLine_1.Agl_Line = 10;
            RightLine_1.Agl_Row = 0;
            RightLine_1.Agl_2_Line = 10;
            RightLine_1.Agl_2_Row = 0;
            RightLine_1.Agl_PointNum = 0;
            RightLine_1.Agl_2_PointNum = 0;
            RightLine_1.BroadWire_Cnt = 0;
            RightLine_1.Straight_Cnt = 0;
            RightLine_1.Turn_Cnt = 0;
            RightLine_1.PointCnt = 0;
            RightLine_1.Err_Cnt = 0;
            RightLine_1.Error = 0;
            RightLine_1.Reserve_Cnt = 0;
            RightLine_1.Rec_Point = 0;
            RightLine_1.RecStrError = 0;
            RightLine_1.AglStrError = 0;
            RightLine_1.Line_k = 0;
            RightLine_1.Line_b = 0;
            //左
            LeftLine_1.StartLine = 0;
            LeftLine_1.EndLine = 0;
            LeftLine_1.Agl_Line = 175;
            LeftLine_1.Agl_Row = 0;
            LeftLine_1.Agl_2_Line = 175;
            LeftLine_1.Agl_2_Row = 0;
            LeftLine_1.Agl_PointNum = 0;
            LeftLine_1.Agl_2_PointNum = 0;
            LeftLine_1.BroadWire_Cnt = 0;
            LeftLine_1.Straight_Cnt = 0;
            LeftLine_1.Turn_Cnt = 0;
            LeftLine_1.PointCnt = 0;
            LeftLine_1.Err_Cnt = 0;
            LeftLine_1.Error = 0;
            LeftLine_1.Reserve_Cnt = 0;
            LeftLine_1.Rec_Point = 0;
            LeftLine_1.RecStrError = 0;
            LeftLine_1.AglStrError = 0;
            LeftLine_1.Line_k = 0;
            LeftLine_1.Line_b = 0;
            //右
            RightLine_2.StartLine = 0;
            RightLine_2.EndLine = 0;
            RightLine_2.Agl_Line = 10;
            RightLine_2.Agl_Row = 0;
            RightLine_2.Agl_2_Line = 10;
            RightLine_2.Agl_2_Row = 0;
            RightLine_2.Agl_PointNum = 0;
            RightLine_2.Agl_2_PointNum = 0;
            RightLine_2.BroadWire_Cnt = 0;
            RightLine_2.Straight_Cnt = 0;
            RightLine_2.Turn_Cnt = 0;
            RightLine_2.PointCnt = 0;
            RightLine_2.Err_Cnt = 0;
            RightLine_2.Error = 0;
            RightLine_2.Reserve_Cnt = 0;
            RightLine_2.Rec_Point = 0;
            RightLine_2.RecStrError = 0;
            RightLine_2.AglStrError = 0;
            RightLine_2.Line_k = 0;
            RightLine_2.Line_b = 0;
            //左
            LeftLine_2.StartLine = 0;
            LeftLine_2.EndLine = 0;
            LeftLine_2.Agl_Line = 175;
            LeftLine_2.Agl_Row = 0;
            LeftLine_2.Agl_2_Line = 175;
            LeftLine_2.Agl_2_Row = 0;
            LeftLine_2.Agl_PointNum = 0;
            LeftLine_2.Agl_2_PointNum = 0;
            LeftLine_2.BroadWire_Cnt = 0;
            LeftLine_2.Straight_Cnt = 0;
            LeftLine_2.Turn_Cnt = 0;
            LeftLine_2.PointCnt = 0;
            LeftLine_2.Err_Cnt = 0;
            LeftLine_2.Error = 0;
            LeftLine_2.Reserve_Cnt = 0;
            LeftLine_2.Rec_Point = 0;
            LeftLine_2.RecStrError = 0;
            LeftLine_2.AglStrError = 0;
            LeftLine_2.Line_k = 0;
            LeftLine_2.Line_b = 0;
            for (x = 0; x < 180; x++)
            {
                RightMargin_1[x].row = 0;
                RightMargin_1[x].line = 0;
                RightMargin_1[x].direction = 0;
                LeftMargin_1[x].row = 0;
                LeftMargin_1[x].line = 185;
                LeftMargin_1[x].direction = 0;
            }
            for (x = 0; x < 150; x++)
            {
                RightMargin_2[x].row = 0;
                RightMargin_2[x].line = 0;
                RightMargin_2[x].direction = 0;
                LeftMargin_2[x].row = 0;
                LeftMargin_2[x].line = 185;
                LeftMargin_2[x].direction = 0;
            }
        }
        void ZebraLine_Init() // 斑马线判断初始化
        {
            //斑马线
            Zebra_Line.StartLine = 0;
            Zebra_Line.EndLine = 0;
            Zebra_Line.Agl_Line = 175;
            Zebra_Line.Agl_Row = 0;
            Zebra_Line.Agl_2_Line = 175;
            Zebra_Line.Agl_2_Row = 0;
            Zebra_Line.Agl_PointNum = 0;
            Zebra_Line.Agl_2_PointNum = 0;
            Zebra_Line.BroadWire_Cnt = 0;
            Zebra_Line.Straight_Cnt = 0;
            Zebra_Line.Turn_Cnt = 0;
            Zebra_Line.PointCnt = 0;
            Zebra_Line.Err_Cnt = 0;
            Zebra_Line.Error = 0;
            Zebra_Line.Reserve_Cnt = 0;
            Zebra_Line.Rec_Point = 0;
            Zebra_Line.RecStrError = 0;
            Zebra_Line.AglStrError = 0;
            Zebra_Line.Line_k = 0;
            Zebra_Line.Line_b = 0;
        }
        void CarLine_Init() // 双车车辆判断初始化
        {
            //右车线
            Car_RightLine.StartLine = 0;
            Car_RightLine.EndLine = 0;
            Car_RightLine.Agl_Line = 10;
            Car_RightLine.Agl_Row = 0;
            Car_RightLine.Agl_2_Line = 10;
            Car_RightLine.Agl_2_Row = 0;
            Car_RightLine.Agl_PointNum = 0;
            Car_RightLine.Agl_2_PointNum = 0;
            Car_RightLine.BroadWire_Cnt = 0;
            Car_RightLine.Straight_Cnt = 0;
            Car_RightLine.Turn_Cnt = 0;
            Car_RightLine.PointCnt = 0;
            Car_RightLine.Err_Cnt = 0;
            Car_RightLine.Error = 0;
            Car_RightLine.Reserve_Cnt = 0;
            Car_RightLine.Rec_Point = 0;
            Car_RightLine.RecStrError = 0;
            Car_RightLine.AglStrError = 0;
            Car_RightLine.Line_k = 0;
            Car_RightLine.Line_b = 0;
            //左车线
            Car_LeftLine.StartLine = 0;
            Car_LeftLine.EndLine = 0;
            Car_LeftLine.Agl_Line = 175;
            Car_LeftLine.Agl_Row = 0;
            Car_LeftLine.Agl_2_Line = 175;
            Car_LeftLine.Agl_2_Row = 0;
            Car_LeftLine.Agl_PointNum = 0;
            Car_LeftLine.Agl_2_PointNum = 0;
            Car_LeftLine.BroadWire_Cnt = 0;
            Car_LeftLine.Straight_Cnt = 0;
            Car_LeftLine.Turn_Cnt = 0;
            Car_LeftLine.PointCnt = 0;
            Car_LeftLine.Err_Cnt = 0;
            Car_LeftLine.Error = 0;
            Car_LeftLine.Reserve_Cnt = 0;
            Car_LeftLine.Rec_Point = 0;
            Car_LeftLine.RecStrError = 0;
            Car_LeftLine.AglStrError = 0;
            Car_LeftLine.Line_k = 0;
            Car_LeftLine.Line_b = 0;
        }
        void CarMargin_Init() // 双车车辆判断初始化
        {
            for (x = 0; x < 50; x++)
            {
                Car_RightMargin[x].row = 0;
                Car_RightMargin[x].line = 0;
                Car_RightMargin[x].direction = 0;
                Car_LeftMargin[x].row = 0;
                Car_LeftMargin[x].line = 185;
                Car_LeftMargin[x].direction = 0;
            }
        }
        void EightRegion_Init() // 八邻域初始化
        {
            LineMargin_Init();
            ZebraLine_Init();
            CarMargin_Init();
            CarLine_Init();
            Record_Flag = 0;
            Error_Flag = 0;
        }
        byte Initial_Seed_Find(Scan_LineType Line_RorL, byte Scan_EndRow) // 种子初始化
        {
            if (Line_RorL == Scan_LineType.LeftType)
            {
                LeftMargin_1[0].row = 0;
                for (OlRow = 1; OlRow < Scan_EndRow; OlRow++)
                {
                    for (OlLine = InitSeedLine; OlLine < 185; OlLine++)
                    {
                        if (J_Pixels[OlRow][OlLine - 1] == white && J_Pixels[OlRow][OlLine] == black)
                        {
                            LeftMargin_1[0].row = OlRow;
                            LeftMargin_1[0].line = OlLine;
                            LeftMargin_1[0].direction = (byte)NC_Direction.NC_equalright;
                            LeftLine_1.StartLine = OlRow;
                            Mask1[OlRow, OlLine] = 1;//标记
                            SetText_1("Init_Seed_1 Find " + Line_RorL + " OlRow: " + OlRow + " OlLine: " + OlLine);
                            //SetText_1("LeftLine_1.StartLine = " + OlRow);
                            return 1;
                        }
                    }
                }
            }
            else
            {
                RightMargin_1[0].row = 0;
                for (OlRow = 1; OlRow < Scan_EndRow; OlRow++)
                {
                    for (OlLine = InitSeedLine; OlLine > 0; OlLine--)
                    {
                        if (J_Pixels[OlRow][OlLine + 1] == white && J_Pixels[OlRow][OlLine] == black)
                        {
                            RightMargin_1[0].row = OlRow;
                            RightMargin_1[0].line = OlLine;
                            RightMargin_1[0].direction = (byte)SC_Direction.SC_equalleft;
                            RightLine_1.StartLine = OlRow;
                            Mask2[OlRow, OlLine] = 1;//标记
                            SetText_1("Init_Seed_1 Find " + Line_RorL + " OlRow: " + OlRow + " OlLine: " + OlLine);
                            //SetText_1("RightLine_1.StartLine = " + OlRow);
                            return 1;
                        }
                    }
                }
            }
            return 0;
        }
        byte Car_Seed_Find(byte ValidData, byte LowerData, byte UpperData) // 寻找另一辆车的扫描种子
        {
            SetText_1(" ");
            SetText_1("//Car_Seed Find  Dec = Vertical " + " Line " + ValidData + " LowerRow: " + LowerData + " UpperRow: " + UpperData);
            for (OlRow = LowerData; OlRow < UpperData; OlRow++)
            {
                if (J_Pixels[OlRow - 1][ValidData] == white && J_Pixels[OlRow][ValidData] == black)
                {
                    Car_LeftMargin[0].row = OlRow;
                    Car_LeftMargin[0].line = ValidData;
                    Car_LeftMargin[0].direction = (byte)NC_Direction.NC_downmiddle;
                    Car_LeftLine.StartLine = OlRow;
                    Mask1[OlRow, ValidData] = 1;//标记

                    Car_RightMargin[0].row = OlRow;
                    Car_RightMargin[0].line = ValidData;
                    Car_RightMargin[0].direction = (byte)SC_Direction.SC_downmiddle;
                    Car_RightLine.StartLine = OlRow;
                    Mask2[OlRow, ValidData] = 1;//标记
                    SetText_1("Car_Seed Find " +  " OlRow: " + OlRow + " OlLine: " + ValidData);
                    return 1;
                }
            }
            SetText_1("! Car Seed Lost Dec = Vertical");
            return 0;
        }
        void Car_RegionScan_NC(Margin Line, uint8_t ScanCnt) // 左车线，八领域扫描，基本操作
        {
            //八邻域 边界追踪
            uint8_t i = 0;
            if (Record_Flag == 0 && Line.row >= 60)
                Record_Flag = 1;
            for (i = 0; i < 8; i++)
            {
                switch ((Line.direction + i) % 8)
                {
                    case 0://右下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line - 1] == black)
                            {
                                Car_RightMargin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Car_RightMargin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Car_RightMargin[ScanCnt + 1].direction = (byte)NC_Direction.NC_downleft;
                                Mask1[Car_RightMargin[ScanCnt + 1].row, Car_RightMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 1://右边的点
                        {
                            if (J_Pixels[Line.row][Line.line - 1] == black)
                            {
                                Car_RightMargin[ScanCnt + 1].row = (byte)(Line.row);
                                Car_RightMargin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Car_RightMargin[ScanCnt + 1].direction = (byte)NC_Direction.NC_downmiddle;
                                Mask1[Car_RightMargin[ScanCnt + 1].row, Car_RightMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                    //Car_RightLine.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 2://右上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line - 1] == black)
                            {
                                Car_RightMargin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Car_RightMargin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Car_RightMargin[ScanCnt + 1].direction = (byte)NC_Direction.NC_downright;
                                Mask1[Car_RightMargin[ScanCnt + 1].row, Car_RightMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 3://正上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line] == black)
                            {
                                Car_RightMargin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Car_RightMargin[ScanCnt + 1].line = (byte)(Line.line);
                                Car_RightMargin[ScanCnt + 1].direction = (byte)NC_Direction.NC_equalright;
                                Mask1[Car_RightMargin[ScanCnt + 1].row, Car_RightMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //Car_RightLine.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 4://左上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line + 1] == black)
                            {
                                Car_RightMargin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Car_RightMargin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Car_RightMargin[ScanCnt + 1].direction = (byte)NC_Direction.NC_upright;
                                Mask1[Car_RightMargin[ScanCnt + 1].row, Car_RightMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 5://正左边的点
                        {
                            if (J_Pixels[Line.row][Line.line + 1] == black)
                            {
                                Car_RightMargin[ScanCnt + 1].row = (byte)(Line.row);
                                Car_RightMargin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Car_RightMargin[ScanCnt + 1].direction = (byte)NC_Direction.NC_upmiddle;
                                Mask1[Car_RightMargin[ScanCnt + 1].row, Car_RightMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //Car_RightLine.Turn_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 6://左下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line + 1] == black)
                            {
                                Car_RightMargin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Car_RightMargin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Car_RightMargin[ScanCnt + 1].direction = (byte)NC_Direction.NC_upleft;
                                Mask1[Car_RightMargin[ScanCnt + 1].row, Car_RightMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 7://正下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line] == black)
                            {
                                Car_RightMargin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Car_RightMargin[ScanCnt + 1].line = (byte)(Line.line);
                                Car_RightMargin[ScanCnt + 1].direction = (byte)NC_Direction.NC_equalleft;
                                Mask1[Car_RightMargin[ScanCnt + 1].row, Car_RightMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //Car_RightLine.Reserve_Cnt++;
                                return;
                            }
                            break;
                        }
                }
            }
        }  //左车线
        void Car_RegionScan_SC(Margin Line, uint8_t ScanCnt) // 右车线，八邻域扫描，基本操作
        {
            //八邻域 边界追踪
            uint8_t i = 0;
            if (Record_Flag == 0 && Line.row >= 60)
                Record_Flag = 1;
            for (i = 0; i < 8; i++)
            {
                switch ((Line.direction + i) % 8)
                {
                    case 0://左下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line + 1] == black)
                            {
                                Car_LeftMargin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Car_LeftMargin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Car_LeftMargin[ScanCnt + 1].direction = (byte)SC_Direction.SC_downright;
                                Mask2[Car_LeftMargin[ScanCnt + 1].row, Car_LeftMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //SetText_1("Car_LeftMargin["+ScanCnt + 1+"].row = "+ Car_LeftMargin[ScanCnt + 1].row + "Car_LeftMargin[" + ScanCnt + 1 + "].line = " + Car_LeftMargin[ScanCnt + 1].line + "Car_LeftMargin[" + ScanCnt + 1 + "].direction = " + Car_LeftMargin[ScanCnt + 1].direction);
                                return;
                            }
                            break;
                        }
                    case 1://左边的点
                        {
                            if (J_Pixels[Line.row][Line.line + 1] == black)
                            {
                                Car_LeftMargin[ScanCnt + 1].row = (byte)(Line.row);
                                Car_LeftMargin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Car_LeftMargin[ScanCnt + 1].direction = (byte)SC_Direction.SC_downmiddle;
                                Mask2[Car_LeftMargin[ScanCnt + 1].row, Car_LeftMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //SetText_1("Car_LeftMargin[" + ScanCnt + 1 + "].row = " + Car_LeftMargin[ScanCnt + 1].row + "Car_LeftMargin[" + ScanCnt + 1 + "].line = " + Car_LeftMargin[ScanCnt + 1].line + "Car_LeftMargin[" + ScanCnt + 1 + "].direction = " + Car_LeftMargin[ScanCnt + 1].direction);
                                //Car_LeftLine.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 2://左上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line + 1] == black)
                            {
                                Car_LeftMargin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Car_LeftMargin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Car_LeftMargin[ScanCnt + 1].direction = (byte)SC_Direction.SC_downleft;
                                Mask2[Car_LeftMargin[ScanCnt + 1].row, Car_LeftMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //SetText_1("Car_LeftMargin[" + ScanCnt + 1 + "].row = " + Car_LeftMargin[ScanCnt + 1].row + "Car_LeftMargin[" + ScanCnt + 1 + "].line = " + Car_LeftMargin[ScanCnt + 1].line + "Car_LeftMargin[" + ScanCnt + 1 + "].direction = " + Car_LeftMargin[ScanCnt + 1].direction);
                                return;
                            }
                            break;
                        }
                    case 3://正上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line] == black)
                            {
                                Car_LeftMargin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Car_LeftMargin[ScanCnt + 1].line = (byte)(Line.line);
                                Car_LeftMargin[ScanCnt + 1].direction = (byte)SC_Direction.SC_equalleft;
                                Mask2[Car_LeftMargin[ScanCnt + 1].row, Car_LeftMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //SetText_1("Car_LeftMargin[" + ScanCnt + 1 + "].row = " + Car_LeftMargin[ScanCnt + 1].row + "Car_LeftMargin[" + ScanCnt + 1 + "].line = " + Car_LeftMargin[ScanCnt + 1].line + "Car_LeftMargin[" + ScanCnt + 1 + "].direction = " + Car_LeftMargin[ScanCnt + 1].direction);
                                //Car_LeftLine.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 4://右上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line - 1] == black)
                            {
                                Car_LeftMargin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Car_LeftMargin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Car_LeftMargin[ScanCnt + 1].direction = (byte)SC_Direction.SC_upleft;
                                Mask2[Car_LeftMargin[ScanCnt + 1].row, Car_LeftMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //SetText_1("Car_LeftMargin[" + ScanCnt + 1 + "].row = " + Car_LeftMargin[ScanCnt + 1].row + "Car_LeftMargin[" + ScanCnt + 1 + "].line = " + Car_LeftMargin[ScanCnt + 1].line + "Car_LeftMargin[" + ScanCnt + 1 + "].direction = " + Car_LeftMargin[ScanCnt + 1].direction);
                                return;
                            }
                            break;
                        }
                    case 5://正右边的点
                        {
                            if (J_Pixels[Line.row][Line.line - 1] == black)
                            {
                                Car_LeftMargin[ScanCnt + 1].row = (byte)(Line.row);
                                Car_LeftMargin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Car_LeftMargin[ScanCnt + 1].direction = (byte)SC_Direction.SC_upmiddle;
                                Mask2[Car_LeftMargin[ScanCnt + 1].row, Car_LeftMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //SetText_1("Car_LeftMargin[" + ScanCnt + 1 + "].row = " + Car_LeftMargin[ScanCnt + 1].row + "Car_LeftMargin[" + ScanCnt + 1 + "].line = " + Car_LeftMargin[ScanCnt + 1].line + "Car_LeftMargin[" + ScanCnt + 1 + "].direction = " + Car_LeftMargin[ScanCnt + 1].direction);
                                //Car_LeftLine.Turn_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 6://右下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line - 1] == black)
                            {
                                Car_LeftMargin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Car_LeftMargin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Car_LeftMargin[ScanCnt + 1].direction = (byte)SC_Direction.SC_upright;
                                //SetText_1("Car_LeftMargin[" + ScanCnt + 1 + "].row = " + Car_LeftMargin[ScanCnt + 1].row + "Car_LeftMargin[" + ScanCnt + 1 + "].line = " + Car_LeftMargin[ScanCnt + 1].line + "Car_LeftMargin[" + ScanCnt + 1 + "].direction = " + Car_LeftMargin[ScanCnt + 1].direction);
                                Mask2[Car_LeftMargin[ScanCnt + 1].row, Car_LeftMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 7://正下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line] == black)
                            {
                                Car_LeftMargin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Car_LeftMargin[ScanCnt + 1].line = (byte)(Line.line);
                                Car_LeftMargin[ScanCnt + 1].direction = (byte)SC_Direction.SC_equalright;
                                //SetText_1("Car_LeftMargin[" + ScanCnt + 1 + "].row = " + Car_LeftMargin[ScanCnt + 1].row + "Car_LeftMargin[" + ScanCnt + 1 + "].line = " + Car_LeftMargin[ScanCnt + 1].line + "Car_LeftMargin[" + ScanCnt + 1 + "].direction = " + Car_LeftMargin[ScanCnt + 1].direction);
                                Mask2[Car_LeftMargin[ScanCnt + 1].row, Car_LeftMargin[ScanCnt + 1].line] = 1;//在图像中标记
                                //Car_LeftLine.Reserve_Cnt++;
                                return;
                            }
                            break;
                        }
                }
            }
        }  //右车线
        void Car_RegionScanLine(Scan_LineType Line_RorL, byte StartPoint, byte TotalPoint) // 车线扫描
        {
            byte Trans_Point = StartPoint;
            //byte ReFind_Flag = 0;
            Record_Flag = 0;
            SetText_1("//Car_Scan: " + Line_RorL + " StartPoint: " + Trans_Point + " TotalPoint: " + TotalPoint);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Car_LeftMargin[0].row == 0 && Car_LeftMargin[0].line == 185)
                {
                    SetText_1("! No Seed .Skip Find");
                    return;
                }
                Car_LeftLine.EndLine = Car_LeftMargin[0].row;
                while (Trans_Point < TotalPoint-1)
                {
                    //SetText_1("Car_LeftMargin["+ Trans_Point+"].row = "+ Car_LeftMargin[Trans_Point].row);
                    Car_RegionScan_SC(Car_LeftMargin[Trans_Point], Trans_Point);
                    Trans_Point++;
                    if ( Car_LeftMargin[Trans_Point].line == 185 || Car_LeftMargin[Trans_Point].line == 0 || Car_LeftMargin[Trans_Point].row == 0
                        || CarReflect_Detect(Scan_LineType.LeftType,  Trans_Point) == 1)
                    {
                        break;
                    }
                    if (Car_LeftLine.EndLine < Car_LeftMargin[Trans_Point].row)
                        Car_LeftLine.EndLine = Car_LeftMargin[Trans_Point].row;
                }
                SetText_1("CarLeftMargin[" + Trans_Point + "].row = " + Car_LeftMargin[Trans_Point].row + " CarLeftMargin[" + Trans_Point + "].line = " + Car_LeftMargin[Trans_Point].line);
                Car_LeftLine.PointCnt = Trans_Point;//记录点的个数  可判断赛道类型等
                //SetText_1("LeftLine_1.EndLine = " + LeftLine_1.EndLine);
            }
            else
            {
                if (Car_RightMargin[0].row == 0 && Car_RightMargin[0].line == 0)
                {
                    SetText_1("! No Seed .Skip Find");
                    return;
                }
                Car_RightLine.EndLine = Car_RightMargin[0].row;
                while (Trans_Point < TotalPoint-1)
                {
                    Car_RegionScan_NC(Car_RightMargin[Trans_Point], Trans_Point);
                    Trans_Point++;
                    if (Car_RightMargin[Trans_Point].line == 185 || Car_RightMargin[Trans_Point].line == 0 || Car_RightMargin[Trans_Point].row == 0
                        || CarReflect_Detect(Scan_LineType.RightType, Trans_Point) == 1)
                    {
                        break;
                    }
                    if (Car_RightLine.EndLine < Car_RightMargin[Trans_Point].row)
                        Car_RightLine.EndLine = Car_RightMargin[Trans_Point].row;
                }
                Car_RightLine.PointCnt = Trans_Point;
                SetText_1("CarRightMargin[" + Trans_Point + "].row = " + Car_RightMargin[Trans_Point].row + " CarRightMargin[" + Trans_Point + "].line = " + Car_RightMargin[Trans_Point].line);
                //SetText_1("RightLine_1.EndLine = " + RightLine_1.EndLine);
            }
            SetText_1("EndPoint " + Trans_Point);
        }
        byte Zebra_Seed_Find(Scan_LineType Line_RorL, Scan_Direction Ver_or_Hor, byte ValidData, byte LowerData, byte UpperData) // 斑马线种子
        {
            SetText_1("//Zebra_Seed Find " + Line_RorL + " Dec " + Ver_or_Hor + " ValidData " + ValidData + " LowerData: " + LowerData + " UpperData: " + UpperData);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Ver_or_Hor == Scan_Direction.Vertical)//垂直寻找下一个种子
                {
                    for (OlRow = LowerData; OlRow < UpperData; OlRow++)
                    {
                        if (J_Pixels[OlRow - 1][ValidData] == white && J_Pixels[OlRow][ValidData] == black)
                        {
                            Zebra_Margin[0].row = OlRow;
                            Zebra_Margin[0].line = ValidData;
                            Zebra_Margin[0].direction = (byte)NC_Direction.NC_downmiddle;
                            Zebra_Line.StartLine = OlRow;
                            Zebra_Line.Agl_Row = OlRow;
                            Zebra_Line.Agl_Line = ValidData;
                            Mask1[OlRow, ValidData] = 1;//标记
                            SetText_1("Zebra_Seed Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + OlRow + " OlLine: " + ValidData);
                            return 1;
                        }
                    }
                }
                else//水平寻找下一个种子
                {
                    for (OlLine = LowerData; OlLine < UpperData; OlLine++)
                    {
                        if (J_Pixels[ValidData][OlLine - 1] == white && J_Pixels[ValidData][OlLine] == black)
                        {
                            Zebra_Margin[0].row = ValidData;
                            Zebra_Margin[0].line = OlLine;
                            Zebra_Margin[0].direction = (byte)NC_Direction.NC_equalright;
                            Zebra_Line.StartLine = ValidData;
                            Zebra_Line.Agl_Row = ValidData;
                            Zebra_Line.Agl_Line = OlLine;
                            Mask1[ValidData, OlLine] = 1;//标记
                            SetText_1("Zebra_Seed Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + ValidData + " OlLine: " + OlLine);
                            return 1;
                        }
                    }
                }
                SetText_1("! Zebra_Left Seed Lost");
            }
            else
            {
                if (Ver_or_Hor == Scan_Direction.Vertical)//垂直寻找下一个种子
                {
                    for (OlRow = LowerData; OlRow < UpperData; OlRow++)
                    {
                        if (J_Pixels[OlRow - 1][ValidData] == white && J_Pixels[OlRow][ValidData] == black)
                        {
                            Zebra_Margin[0].row = OlRow;
                            Zebra_Margin[0].line = ValidData;
                            Zebra_Margin[0].direction = (byte)SC_Direction.SC_downmiddle;
                            Zebra_Line.StartLine = OlRow;
                            Zebra_Line.Agl_Row = OlRow;
                            Zebra_Line.Agl_Line = ValidData;
                            Mask2[OlRow, ValidData] = 1;//标记
                            SetText_1("Zebra_Seed Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + OlRow + " OlLine: " + ValidData);
                            return 1;
                        }
                    }
                }
                else//水平寻找下一个种子
                {
                    for (OlLine = UpperData; OlLine > LowerData; OlLine--)
                    {
                        if (J_Pixels[ValidData][OlLine + 1] == white && J_Pixels[ValidData][OlLine] == black)
                        {
                            Zebra_Margin[0].row = ValidData;
                            Zebra_Margin[0].line = OlLine;
                            Zebra_Margin[0].direction = (byte)SC_Direction.SC_equalleft;
                            Zebra_Line.StartLine = ValidData;
                            Zebra_Line.Agl_Row = ValidData;
                            Zebra_Line.Agl_Line = OlLine;
                            Mask2[ValidData, OlLine] = 1;//标记
                            SetText_1("Zebra_Seed Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + ValidData + " OlLine: " + OlLine);
                            return 1;
                        }
                    }
                }
                SetText_1("! Zebra_Right Seed Lost");
            }
            return 0;
        }
        void Zebra_RegionScan_NC(Margin Line, uint8_t ScanCnt,byte m) // 斑马线八邻域扫描，基本操作
        {
            //八邻域 边界追踪
            uint8_t i = 0;
            if (m == 0)
            {
                if (Line.line < Zebra_Line.Agl_Line)
                {
                    Zebra_Line.Agl_Row = Line.row;
                    Zebra_Line.Agl_Line = Line.line;
                    //SetText_1("Zebra_Line Border.row = " + Line.row + " Border.line = " + Line.line);
                }
            }
            else if(m==1)
            {
                if (Line.line > Zebra_Line.Agl_Line)
                {
                    Zebra_Line.Agl_Row = Line.row;
                    Zebra_Line.Agl_Line = Line.line;
                    //SetText_1("Zebra_Line Border.row = " + Line.row + " Border.line = " + Line.line);
                }
            }
            for (i = 0; i < 8; i++)
            {
                switch ((Line.direction + i) % 8)
                {
                    case 0://右下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line - 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)NC_Direction.NC_downleft;
                                Mask1[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 1://右边的点
                        {
                            if (J_Pixels[Line.row][Line.line - 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)NC_Direction.NC_downmiddle;
                                Mask1[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 2://右上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line - 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)NC_Direction.NC_downright;
                                Mask1[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 3://正上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)NC_Direction.NC_equalright;
                                Mask1[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 4://左上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line + 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)NC_Direction.NC_upright;
                                Mask1[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 5://正左边的点
                        {
                            if (J_Pixels[Line.row][Line.line + 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)NC_Direction.NC_upmiddle;
                                Mask1[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 6://左下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line + 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)NC_Direction.NC_upleft;
                                Mask1[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 7://正下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)NC_Direction.NC_equalleft;
                                Mask1[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                }
            }
        }
        void Zebra_RegionScan_SC(Margin Line, uint8_t ScanCnt,byte m) // 斑马线八邻域扫描，基本操作
        {
            //八邻域 边界追踪
            uint8_t i = 0;
            if (m == 0)
            {
                if (Line.line > Zebra_Line.Agl_Line)
                {
                    Zebra_Line.Agl_Row = Line.row;
                    Zebra_Line.Agl_Line = Line.line;
                    //SetText_1("Zebra_Line Border.row = " + Line.row + " Top.line = " + Line.line);
                }
            }
            else if(m==1)
            {
                if (Line.line < Zebra_Line.Agl_Line)
                {
                    Zebra_Line.Agl_Row = Line.row;
                    Zebra_Line.Agl_Line = Line.line;
                    //SetText_1("Zebra_Line Border.row = " + Line.row + " Top.line = " + Line.line);
                }
            }
            for (i = 0; i < 8; i++)
            {
                switch ((Line.direction + i) % 8)
                {
                    case 0://左下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line + 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)SC_Direction.SC_downright;
                                Mask2[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 1://左边的点
                        {
                            if (J_Pixels[Line.row][Line.line + 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)SC_Direction.SC_downmiddle;
                                Mask2[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 2://左上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line + 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line + 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)SC_Direction.SC_downleft;
                                Mask2[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 3://正上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)SC_Direction.SC_equalleft;
                                Mask2[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 4://右上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line - 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row + 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)SC_Direction.SC_upleft;
                                Mask2[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 5://正右边的点
                        {
                            if (J_Pixels[Line.row][Line.line - 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)SC_Direction.SC_upmiddle;
                                Mask2[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 6://右下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line - 1] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line - 1);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)SC_Direction.SC_upright;
                                Mask2[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 7://正下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line] == black)
                            {
                                Zebra_Margin[ScanCnt + 1].row = (byte)(Line.row - 1);
                                Zebra_Margin[ScanCnt + 1].line = (byte)(Line.line);
                                Zebra_Margin[ScanCnt + 1].direction = (byte)SC_Direction.SC_equalright;
                                Mask2[Zebra_Margin[ScanCnt + 1].row, Zebra_Margin[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                }
            }
        }
        byte Zebra_RegionScanLine(Scan_LineType Line_RorL, byte StartPoint, byte TotalPoint,byte m) // 斑马线八邻域扫描
        {
            byte Trans_Point = StartPoint;
            SetText_1("//Scan Zebra: " + Line_RorL + " StartPoint: " + Trans_Point + " TotalPoint: " + TotalPoint);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Zebra_Margin[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Find");
                    return 0;
                }
                //SetText_1("Zebra_Margin[0].row = "+ Zebra_Margin[0].row + " Zebra_Margin[0].line = "+ Zebra_Margin[0].line);
                Zebra_Line.EndLine = Zebra_Margin[0].row;
                while (Trans_Point < TotalPoint)
                {
                    //SetText_1("Zebra_Margin["+ Trans_Point+"].row = "+ Zebra_Margin[Trans_Point].row);
                    Zebra_RegionScan_NC(Zebra_Margin[Trans_Point], Trans_Point,m);
                    Trans_Point++;
                    if (Zebra_Margin[Trans_Point].row == 69 || Zebra_Margin[Trans_Point].line == 185 || Zebra_Margin[Trans_Point].line == 0 )
                    {
                        SetText_1("Wrong Zebra Margin!");
                        return 0;
                    }
                    if(Zebra_Margin[Trans_Point].row == Zebra_Margin[0].row && Zebra_Margin[Trans_Point].line == Zebra_Margin[0].line)
                    {
                        SetText_1("Complete Zebra! Point = "+ Trans_Point);
                        return 1;
                    }
                    if(Zebra_Margin[Trans_Point].row == 0)
                    {
                        Zebra_Line.Agl_Line = Zebra_Margin[0].line;
                        Zebra_Line.Agl_Row = Zebra_Margin[0].row;
                        SetText_1("Half Zebra! Point = " + Trans_Point);
                        return 1;
                    }
                    //SetText_1("Zebra_Margin["+Trans_Point+"].Row = " + Zebra_Margin[Trans_Point].row+ " Zebra_Margin["+Trans_Point+"].Line = "+ Zebra_Margin[Trans_Point].line);
                }
                //Zebra_Line.PointCnt = Trans_Point;//记录点的个数  可判断赛道类型等
                //SetText_1("Zebra_Line.EndLine = " + Zebra_Line.EndLine);
            }
            else
            {
                if (Zebra_Margin[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Find");
                    return 0;
                }
                Zebra_Line.EndLine = Zebra_Margin[0].row;
                while (Trans_Point < TotalPoint)
                {
                    Zebra_RegionScan_SC(Zebra_Margin[Trans_Point], Trans_Point,m);
                    Trans_Point++;
                    if (Zebra_Margin[Trans_Point].row == 69 || Zebra_Margin[Trans_Point].line == 185 || Zebra_Margin[Trans_Point].line == 0 )
                    {
                        SetText_1("Wrong Zebra Margin!");
                        return 0;
                    }
                    if (Zebra_Margin[Trans_Point].row == Zebra_Margin[0].row && Zebra_Margin[Trans_Point].line == Zebra_Margin[0].line)
                    {
                        SetText_1("Complete Zebra! Point = " + Trans_Point);
                        return 1;
                    }
                    if (Zebra_Margin[Trans_Point].row == 0)
                    {
                        Zebra_Line.Agl_Line = Zebra_Margin[0].line;
                        Zebra_Line.Agl_Row = Zebra_Margin[0].row;
                        SetText_1("Half Zebra! Point = " + Trans_Point);
                        return 1;
                    }
                }
                //Zebra_Line.PointCnt = Trans_Point;
                //SetText_1("Zebra_Line.EndLine = " + Zebra_Line.EndLine);
            }
            SetText_1("Wrong Zebra Margin!");
            return 0;
            //SetText_1("EndPoint " + Trans_Point);
        }
        byte Traverse_CarAgl(Scan_LineType Line_RorL, byte EndRow) // 遍历找到扫描得到的车邻域边线，寻找拐点
        {
            //byte Trans_Point = StartPoint;
            byte Trans_Point = 1;
            SetText_1("//Traverse_Car_Agl: " + Line_RorL + " StartPoint: " + Trans_Point + " EndRow = " + EndRow);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Car_LeftMargin[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Traverse");
                    return 0;
                }
                for (; Trans_Point < Car_LeftLine.PointCnt; Trans_Point++)//  这一步可以优化 第一次扫线是可以进行判断  算了不优化了
                {
                    if (Car_LeftMargin[Trans_Point].row >= EndRow)
                    {
                        SetText_1("! No Car LeftAngley");
                        return 0;
                    }
                    if (Car_LeftMargin[Trans_Point].direction == (byte)SC_Direction.SC_upmiddle)
                    {
                        Car_LeftLine.Turn_Cnt++;
                    }
                    if (Car_LeftLine.Agl_Row == 0)
                    {
                        if (Car_LeftMargin[Trans_Point].direction == (byte)SC_Direction.SC_downmiddle)
                        {
                            Car_LeftLine.BroadWire_Cnt++;
                            //SetText_1("Broad Margin Car_LeftMargin["+Trans_Point+"].row = " + Car_LeftMargin[Trans_Point].row+ " Car_LeftMargin[" + Trans_Point + "].line = "+ Car_LeftMargin[Trans_Point].line);
                        }
                        else if (Car_LeftMargin[Trans_Point].direction == (byte)SC_Direction.SC_equalleft)//通过判断下一个点的位置来判断
                        {
                            SetText_1("* Car LeftAngley 1 :" + "   row:" + Car_LeftMargin[Trans_Point - 1].row + " line" + Car_LeftMargin[Trans_Point - 1].line);
                            //记录角点坐标
                            Car_LeftLine.Agl_Row = Car_LeftMargin[Trans_Point - 1].row;
                            Car_LeftLine.Agl_Line = Car_LeftMargin[Trans_Point - 1].line;
                            Car_LeftLine.Agl_PointNum = (byte)(Trans_Point - 1);
                        }
                    }
                    else
                    {
                        if (Car_LeftMargin[Trans_Point].direction == (byte)SC_Direction.SC_equalleft)
                        {
                            Car_LeftLine.Straight_Cnt++;
                            //SetText_1("Straight Margin Car_LeftMargin["+Trans_Point+"].row = " + Car_LeftMargin[Trans_Point].row+ " Car_LeftMargin[" + Trans_Point + "].line = "+ Car_LeftMargin[Trans_Point].line);
                        }
                        if (Car_LeftMargin[Trans_Point].direction == (byte)SC_Direction.SC_upmiddle || Car_LeftMargin[Trans_Point].direction == (byte)SC_Direction.SC_downmiddle)
                        {
                            SetText_1("* Car LeftAngley 2 :" + "   row:" + Car_LeftMargin[Trans_Point - 1].row + " line" + Car_LeftMargin[Trans_Point - 1].line);
                            //记录角点坐标
                            Car_LeftLine.Agl_2_Row = Car_LeftMargin[Trans_Point - 1].row;
                            Car_LeftLine.Agl_2_Line = Car_LeftMargin[Trans_Point - 1].line;
                            Car_LeftLine.Agl_2_PointNum = (byte)(Trans_Point - 1);
                            return 1;
                        }
                    }
                }
            }
            if (Line_RorL == Scan_LineType.RightType)
            {
                if (Car_RightMargin[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Traverse");
                    return 0;
                }
                for (; Trans_Point < Car_RightLine.PointCnt; Trans_Point++)
                {
                    //SetText_1("Car_RightMargin["+Trans_Point+"].row = "+ Car_RightMargin[Trans_Point].row + " Car_RightMargin["+Trans_Point+"].line = "+ Car_RightMargin[Trans_Point].row);
                    if (Car_RightMargin[Trans_Point].row >= EndRow)
                    {
                        SetText_1("! No Car RightAngley");
                        return 0;
                    }
                    if (Car_RightMargin[Trans_Point].direction == (byte)NC_Direction.NC_upmiddle)
                    {
                        Car_RightLine.Turn_Cnt++;
                    }
                    if (Car_RightLine.Agl_Row == 0)
                    {
                        if (Car_RightMargin[Trans_Point].direction == (byte)NC_Direction.NC_downmiddle)
                        {
                            Car_RightLine.BroadWire_Cnt++;
                            //SetText_1("Broad Margin Car_RightMargin[" + Trans_Point + "].row = " + Car_RightMargin[Trans_Point].row + " Car_RightMargin[" + Trans_Point + "].line = " + Car_RightMargin[Trans_Point].line);
                        }
                        else if (Car_RightMargin[Trans_Point].direction == (byte)NC_Direction.NC_equalright)//通过判断下一个点的位置来判断
                        {
                            SetText_1("* Car RightAngley 1 :" + "   row:" + Car_RightMargin[Trans_Point - 1].row + " line" + Car_RightMargin[Trans_Point - 1].line);
                            Car_RightLine.Agl_Row = Car_RightMargin[Trans_Point - 1].row;
                            Car_RightLine.Agl_Line = Car_RightMargin[Trans_Point - 1].line;
                            Car_RightLine.Agl_PointNum = (byte)(Trans_Point - 1);
                        }
                    }
                    else
                    {
                        if (Car_RightMargin[Trans_Point].direction == (byte)NC_Direction.NC_equalright)
                        {
                            Car_RightLine.Straight_Cnt++;
                        }
                        if (Car_RightMargin[Trans_Point].direction == (byte)NC_Direction.NC_upmiddle||Car_RightMargin[Trans_Point].direction == (byte)NC_Direction.NC_downmiddle)//通过判断下一个点的位置来判断
                        {
                            SetText_1("* Car RightAngley 2 :" + "   row:" + Car_RightMargin[Trans_Point - 1].row + " line" + Car_RightMargin[Trans_Point - 1].line);
                            Car_RightLine.Agl_2_Row = Car_RightMargin[Trans_Point - 1].row;
                            Car_RightLine.Agl_2_Line = Car_RightMargin[Trans_Point - 1].line;
                            Car_RightLine.Agl_2_PointNum = (byte)(Trans_Point - 1);
                            return 1;
                        }
                    }
                }
            }
            return 0;
        }
        byte NewSeedFind_1(Scan_LineType Line_RorL, Scan_Direction Ver_or_Hor, byte ValidData, byte LowerData, byte UpperData)// 种子寻找
        {
            SetText_1("//Seed_1 Find " + Line_RorL + " Dec " + Ver_or_Hor + " ValidData " + ValidData + " LowerData: " + LowerData + " UpperData: " + UpperData);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Ver_or_Hor == Scan_Direction.Vertical)//垂直寻找下一个种子
                {
                    for (OlRow = LowerData; OlRow < UpperData&& OlRow<68; OlRow++)
                    {
                        if (J_Pixels[OlRow - 1][ValidData] == white && J_Pixels[OlRow][ValidData] == black)
                        {
                            LeftMargin_1[0].row = OlRow;
                            LeftMargin_1[0].line = ValidData;
                            LeftMargin_1[0].direction = (byte)NC_Direction.NC_downmiddle;
                            LeftLine_1.StartLine = OlRow;
                            LeftLine_1.Rec_Point = 0;
                            Mask1[OlRow, ValidData] = 1;//标记
                            SetText_1("Seed_1 Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + OlRow + " OlLine: " + ValidData);
                            return 1;
                        }
                    }
                }
                else//水平寻找下一个种子
                {
                    for (OlLine = LowerData; OlLine < UpperData && OlLine < 184; OlLine++)
                    {
                        if (J_Pixels[ValidData][OlLine - 1] == white && J_Pixels[ValidData][OlLine] == black)
                        {
                            LeftMargin_1[0].row = ValidData;
                            LeftMargin_1[0].line = OlLine;
                            LeftMargin_1[0].direction = (byte)NC_Direction.NC_equalright;
                            LeftLine_1.StartLine = ValidData;
                            LeftLine_1.Rec_Point = 0;
                            Mask1[ValidData, OlLine] = 1;//标记
                            SetText_1("Seed_1 Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + ValidData + " OlLine: " + OlLine);
                            return 1;
                        }
                    }
                }
                SetText_1("! Left 1 Seed Lost");
            }
            else
            {
                if (Ver_or_Hor == Scan_Direction.Vertical)//垂直寻找下一个种子
                {
                    for (OlRow = LowerData; OlRow < UpperData && OlRow < 68; OlRow++)
                    {
                        if (J_Pixels[OlRow - 1][ValidData] == white && J_Pixels[OlRow][ValidData] == black)
                        {
                            RightMargin_1[0].row = OlRow;
                            RightMargin_1[0].line = ValidData;
                            RightMargin_1[0].direction = (byte)SC_Direction.SC_downmiddle;
                            RightLine_1.StartLine = OlRow;
                            RightLine_1.Rec_Point = 0;
                            Mask2[OlRow, ValidData] = 1;//标记
                            SetText_1("Seed_1 Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + OlRow + " OlLine: " + ValidData);
                            return 1;
                        }
                    }
                }
                else//水平寻找下一个种子
                {
                    for (OlLine = UpperData; OlLine > LowerData && OlLine > 1; OlLine--)
                    {
                        if (J_Pixels[ValidData][OlLine + 1] == white && J_Pixels[ValidData][OlLine] == black)
                        {
                            RightMargin_1[0].row = ValidData;
                            RightMargin_1[0].line = OlLine;
                            RightMargin_1[0].direction = (byte)SC_Direction.SC_equalleft;
                            RightLine_1.StartLine = ValidData;
                            RightLine_1.Rec_Point = 0;
                            Mask2[ValidData, OlLine] = 1;//标记
                            SetText_1("Seed_1 Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + ValidData + " OlLine: " + OlLine);
                            return 1;
                        }
                    }
                }
                SetText_1("! Right 1 Seed Lost");
            }
            return 0;
        }
        byte NewSeedFind_2(Scan_LineType Line_RorL, Scan_Direction Ver_or_Hor, byte ValidData, byte LowerData, byte UpperData)// 种子寻找
        {
            SetText_1("//Seed_2 Find " + Line_RorL + " Dec " + Ver_or_Hor + " ValidData " + ValidData + " LowerData: " + LowerData + " UpperData: " + UpperData);
            //if (LowerData == 3)
            //{
            //    SetText_1("! Lost Agl_1 " + Line_RorL + " Skip Find Seed_2 " + " Dec " + Ver_or_Hor + " OlRow: " + OlRow + " OlLine: " + ValidData);
            //    return 0;
            //}
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Ver_or_Hor == Scan_Direction.Vertical)//垂直寻找下一个种子
                {
                    if (ValidData > 184)
                        ValidData = 184;
                    //ValidData = (byte)(ValidData + 10) < 184 ? (byte)(ValidData + 10) : (byte)184;
                    for (OlRow = LowerData; OlRow < UpperData && OlRow < 68; OlRow++)
                    {
                        if (J_Pixels[OlRow - 1][ValidData] == white && J_Pixels[OlRow][ValidData] == black)
                        {
                            LeftMargin_2[0].row = OlRow;
                            LeftMargin_2[0].line = ValidData;
                            LeftMargin_2[0].direction = (byte)NC_Direction.NC_downmiddle;
                            LeftLine_2.StartLine = OlRow;
                            LeftLine_2.Rec_Point = 0;
                            Mask1[OlRow, ValidData] = 1;//标记
                            SetText_1("Seed_2 Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + OlRow + " OlLine: " + ValidData);
                            //SetText_1("LeftLine_2.StartLine = " + LeftLine_2.StartLine);
                            return 1;
                        }
                        else if(J_Pixels[OlRow - 1][ValidData] == black && J_Pixels[OlRow][ValidData] == black)
                        {
                            SetText_1("!Wrong Path To Find Left Seed 2");
                            return 0;
                        }
                    }
                }
                else//水平寻找下一个种子
                {
                    if (ValidData > 68)
                        ValidData = 68;
                    for (OlLine = LowerData; OlLine < UpperData && OlLine <184; OlLine++)
                    {
                        if (J_Pixels[ValidData][OlLine - 1] == white && J_Pixels[ValidData][OlLine] == black)
                        {
                            LeftMargin_2[0].row = ValidData;
                            LeftMargin_2[0].line = OlLine;
                            LeftMargin_2[0].direction = (byte)NC_Direction.NC_equalright;
                            LeftLine_2.StartLine = ValidData;
                            LeftLine_2.Rec_Point = 0;
                            Mask1[ValidData, OlLine] = 1;//标记
                            SetText_1("Seed_2 Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + ValidData + " OlLine: " + OlLine);
                            //SetText_1("LeftLine_2.StartLine = " + LeftLine_2.StartLine);
                            return 1;
                        }
                    }
                }
                SetText_1("! Left 2 Seed Lost");
            }
            else
            {
                if (Ver_or_Hor == Scan_Direction.Vertical)//垂直寻找下一个种子
                {
                    if (ValidData > 184)
                        ValidData = 184;
                    //ValidData = ValidData > 11 ? (byte)(ValidData - 10) : (byte)1;
                    for (OlRow = LowerData; OlRow < UpperData && OlRow < 68; OlRow++)
                    {
                        if (J_Pixels[OlRow - 1][ValidData] == white && J_Pixels[OlRow][ValidData] == black)
                        {
                            RightMargin_2[0].row = OlRow;
                            RightMargin_2[0].line = ValidData;
                            RightMargin_2[0].direction = (byte)SC_Direction.SC_downmiddle;
                            RightLine_2.StartLine = OlRow;
                            RightLine_2.Rec_Point = 0;
                            Mask2[OlRow, ValidData] = 1;//标记
                            SetText_1("Seed_2 Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + OlRow + " OlLine: " + ValidData);
                            //SetText_1("RightLine_2.StartLine = " + RightLine_2.StartLine);
                            return 1;
                        }
                        else if(J_Pixels[OlRow - 1][ValidData] == black && J_Pixels[OlRow][ValidData] == black)
                        {
                            SetText_1("!Wrong Path To Find Right Seed 2");
                            return 0;
                        }
                    }
                }
                else//水平寻找下一个种子
                {
                    if (ValidData > 68)
                        ValidData = 68;
                    for (OlLine = UpperData; OlLine > LowerData && OlLine > 1; OlLine--)
                    {
                        if (J_Pixels[ValidData][OlLine + 1] == white && J_Pixels[ValidData][OlLine] == black)
                        {
                            RightMargin_2[0].row = ValidData;
                            RightMargin_2[0].line = OlLine;
                            RightMargin_2[0].direction = (byte)SC_Direction.SC_equalleft;
                            RightLine_2.StartLine = ValidData;
                            RightLine_2.Rec_Point = 0;
                            Mask2[ValidData, OlLine] = 1;//标记
                            SetText_1("Seed_2 Find " + Line_RorL + " Dec " + Ver_or_Hor + " OlRow: " + ValidData + " OlLine: " + OlLine);
                            //SetText_1("RightLine_2.StartLine = " + RightLine_2.StartLine);
                            return 1;
                        }
                    }
                }
                SetText_1("! Right 2 Seed Lost");
            }
            return 0;
        }
        void NewSeedSet_1(Scan_LineType Line_RorL, byte row, byte line) // 设置种子
        {
            if (Line_RorL == Scan_LineType.LeftType)
            {
                LeftMargin_1[0].row = row;
                LeftMargin_1[0].line = line;
                LeftMargin_1[0].direction = (byte)NC_Direction.NC_downmiddle;
                LeftLine_1.StartLine = OlRow;
                Mask1[row, line] = 1;//标记
            }
            else
            {
                RightMargin_1[0].row = row;
                RightMargin_1[0].line = line;
                RightMargin_1[0].direction = (byte)SC_Direction.SC_downmiddle;
                RightLine_1.StartLine = OlRow;
                Mask2[row, line] = 1;//标记
            }
            SetText_1("//Seed_1 Set " + Line_RorL + " Row " + row + " Line: " + line);
        }
        void NewSeedSet_2(Scan_LineType Line_RorL, byte row, byte line) // 设置种子
        {
            if (Line_RorL == Scan_LineType.LeftType)
            {
                LeftMargin_2[0].row = row;
                LeftMargin_2[0].line = line;
                LeftMargin_2[0].direction = (byte)NC_Direction.NC_downmiddle;
                LeftLine_2.StartLine = OlRow;
                Mask1[row, line] = 1;//标记
            }
            else
            {
                RightMargin_2[0].row = row;
                RightMargin_2[0].line = line;
                RightMargin_2[0].direction = (byte)SC_Direction.SC_downmiddle;
                RightLine_2.StartLine = OlRow;
                Mask2[row, line] = 1;//标记
            }
            SetText_1("//Seed_2 Set " + Line_RorL + " Row " + row + " Line: " + line);
        }
        void EightRegionScan_1_NC(Margin Line, uint8_t ScanCnt) // 八邻域扫描，左，基本操作
        {
            //八邻域 边界追踪
            uint8_t i = 0;
            if (Record_Flag == 0 && Line.row >= 60)
            {
                if (LeftLine_1.Rec_Point == 0)
                    LeftLine_1.Rec_Point = (byte)(ScanCnt == 0 ? 1 : ScanCnt);
                SetText_1("Rec Line.row = "+ Line.row+ " Line.line = "+ Line.line+ " LeftLine_1.Rec_Point = "+ LeftLine_1.Rec_Point);
                Record_Flag = 1;
            }
            for (i = 0; i < 8; i++)
            {
                switch ((Line.direction + i) % 8)
                {
                    case 0://右下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line - 1] == black)
                            {
                                LeftMargin_1[ScanCnt + 1].row = (byte)(Line.row - 1);
                                LeftMargin_1[ScanCnt + 1].line = (byte)(Line.line - 1);
                                LeftMargin_1[ScanCnt + 1].direction = (byte)NC_Direction.NC_downleft;
                                Mask1[LeftMargin_1[ScanCnt + 1].row, LeftMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 1://右边的点
                        {
                            if (J_Pixels[Line.row][Line.line - 1] == black)
                            {
                                LeftMargin_1[ScanCnt + 1].row = (byte)(Line.row);
                                LeftMargin_1[ScanCnt + 1].line = (byte)(Line.line - 1);
                                LeftMargin_1[ScanCnt + 1].direction = (byte)NC_Direction.NC_downmiddle;
                                Mask1[LeftMargin_1[ScanCnt + 1].row, LeftMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                if (Record_Flag == 0)
                                    LeftLine_1.Turn_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 2://右上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line - 1] == black)
                            {
                                LeftMargin_1[ScanCnt + 1].row = (byte)(Line.row + 1);
                                LeftMargin_1[ScanCnt + 1].line = (byte)(Line.line - 1);
                                LeftMargin_1[ScanCnt + 1].direction = (byte)NC_Direction.NC_downright;
                                Mask1[LeftMargin_1[ScanCnt + 1].row, LeftMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                         //记录赛道信息
                                if (Record_Flag == 0)
                                    LeftLine_1.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 3://正上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line] == black)
                            {
                                LeftMargin_1[ScanCnt + 1].row = (byte)(Line.row + 1);
                                LeftMargin_1[ScanCnt + 1].line = (byte)(Line.line);
                                LeftMargin_1[ScanCnt + 1].direction = (byte)NC_Direction.NC_equalright;
                                Mask1[LeftMargin_1[ScanCnt + 1].row, LeftMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                         //记录赛道信息
                                if (Record_Flag == 0)
                                    LeftLine_1.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 4://左上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line + 1] == black)
                            {
                                LeftMargin_1[ScanCnt + 1].row = (byte)(Line.row + 1);
                                LeftMargin_1[ScanCnt + 1].line = (byte)(Line.line + 1);
                                LeftMargin_1[ScanCnt + 1].direction = (byte)NC_Direction.NC_upright;
                                Mask1[LeftMargin_1[ScanCnt + 1].row, LeftMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 5://正左边的点
                        {
                            if (J_Pixels[Line.row][Line.line + 1] == black)
                            {
                                LeftMargin_1[ScanCnt + 1].row = (byte)(Line.row);
                                LeftMargin_1[ScanCnt + 1].line = (byte)(Line.line + 1);
                                LeftMargin_1[ScanCnt + 1].direction = (byte)NC_Direction.NC_upmiddle;
                                Mask1[LeftMargin_1[ScanCnt + 1].row, LeftMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                         //记录赛道信息
                                if (Record_Flag == 0)
                                    LeftLine_1.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 6://左下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line + 1] == black)
                            {
                                LeftMargin_1[ScanCnt + 1].row = (byte)(Line.row - 1);
                                LeftMargin_1[ScanCnt + 1].line = (byte)(Line.line + 1);
                                LeftMargin_1[ScanCnt + 1].direction = (byte)NC_Direction.NC_upleft;
                                Mask1[LeftMargin_1[ScanCnt + 1].row, LeftMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                         //记录赛道信息
                                if (Record_Flag == 0)
                                    LeftLine_1.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 7://正下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line] == black)
                            {
                                LeftMargin_1[ScanCnt + 1].row = (byte)(Line.row - 1);
                                LeftMargin_1[ScanCnt + 1].line = (byte)(Line.line);
                                LeftMargin_1[ScanCnt + 1].direction = (byte)NC_Direction.NC_equalleft;
                                Mask1[LeftMargin_1[ScanCnt + 1].row, LeftMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                         //记录赛道信息
                                if (Record_Flag == 0)
                                    LeftLine_1.Reserve_Cnt++;
                                return;
                            }
                            break;
                        }
                }
            }
        }  //左边线
        void EightRegionScan_1_SC(Margin Line, uint8_t ScanCnt) // 八邻域扫描，右，基本操作
        {
            //八邻域 边界追踪
            uint8_t i = 0;
            if (Record_Flag == 0 && Line.row >= 60)
            {
                if (RightLine_1.Rec_Point == 0)
                    RightLine_1.Rec_Point = (byte)(ScanCnt == 0 ? 1 : ScanCnt);;
                SetText_1("Rec Line.row = " + Line.row + " Line.line = " + Line.line+ " RightLine_1.Rec_Point = "+ RightLine_1.Rec_Point);
                Record_Flag = 1;
            }
            for (i = 0; i < 8; i++)
            {
                switch ((Line.direction + i) % 8)
                {
                    case 0://左下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line + 1] == black)
                            {
                                RightMargin_1[ScanCnt + 1].row = (byte)(Line.row - 1);
                                RightMargin_1[ScanCnt + 1].line = (byte)(Line.line + 1);
                                RightMargin_1[ScanCnt + 1].direction = (byte)SC_Direction.SC_downright;
                                Mask2[RightMargin_1[ScanCnt + 1].row, RightMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 1://左边的点
                        {
                            if (J_Pixels[Line.row][Line.line + 1] == black)
                            {
                                RightMargin_1[ScanCnt + 1].row = (byte)(Line.row);
                                RightMargin_1[ScanCnt + 1].line = (byte)(Line.line + 1);
                                RightMargin_1[ScanCnt + 1].direction = (byte)SC_Direction.SC_downmiddle;
                                Mask2[RightMargin_1[ScanCnt + 1].row, RightMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                if (Record_Flag == 0)
                                    RightLine_1.Turn_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 2://左上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line + 1] == black)
                            {
                                RightMargin_1[ScanCnt + 1].row = (byte)(Line.row + 1);
                                RightMargin_1[ScanCnt + 1].line = (byte)(Line.line + 1);
                                RightMargin_1[ScanCnt + 1].direction = (byte)SC_Direction.SC_downleft;
                                Mask2[RightMargin_1[ScanCnt + 1].row, RightMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                           //记录赛道信息           
                                if (Record_Flag == 0)
                                    RightLine_1.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 3://正上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line] == black)
                            {
                                RightMargin_1[ScanCnt + 1].row = (byte)(Line.row + 1);
                                RightMargin_1[ScanCnt + 1].line = (byte)(Line.line);
                                RightMargin_1[ScanCnt + 1].direction = (byte)SC_Direction.SC_equalleft;
                                Mask2[RightMargin_1[ScanCnt + 1].row, RightMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                           //记录赛道信息
                                if (Record_Flag == 0)
                                    RightLine_1.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 4://右上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line - 1] == black)
                            {
                                RightMargin_1[ScanCnt + 1].row = (byte)(Line.row + 1);
                                RightMargin_1[ScanCnt + 1].line = (byte)(Line.line - 1);
                                RightMargin_1[ScanCnt + 1].direction = (byte)SC_Direction.SC_upleft;
                                Mask2[RightMargin_1[ScanCnt + 1].row, RightMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 5://正右边的点
                        {
                            if (J_Pixels[Line.row][Line.line - 1] == black)
                            {
                                RightMargin_1[ScanCnt + 1].row = (byte)(Line.row);
                                RightMargin_1[ScanCnt + 1].line = (byte)(Line.line - 1);
                                RightMargin_1[ScanCnt + 1].direction = (byte)SC_Direction.SC_upmiddle;
                                Mask2[RightMargin_1[ScanCnt + 1].row, RightMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                           //记录赛道信息
                                if (Record_Flag == 0)
                                    RightLine_1.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 6://右下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line - 1] == black)
                            {
                                RightMargin_1[ScanCnt + 1].row = (byte)(Line.row - 1);
                                RightMargin_1[ScanCnt + 1].line = (byte)(Line.line - 1);
                                RightMargin_1[ScanCnt + 1].direction = (byte)SC_Direction.SC_upright;
                                Mask2[RightMargin_1[ScanCnt + 1].row, RightMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                           //记录赛道信息
                                if (Record_Flag == 0)
                                    RightLine_1.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 7://正下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line] == black)
                            {
                                RightMargin_1[ScanCnt + 1].row = (byte)(Line.row - 1);
                                RightMargin_1[ScanCnt + 1].line = (byte)(Line.line);
                                RightMargin_1[ScanCnt + 1].direction = (byte)SC_Direction.SC_equalright;
                                Mask2[RightMargin_1[ScanCnt + 1].row, RightMargin_1[ScanCnt + 1].line] = 1;//在图像中标记
                                                                                                           //记录赛道信息
                                if (Record_Flag == 0)
                                    RightLine_1.Reserve_Cnt++;
                                return;
                            }
                            break;
                        }
                }
            }
        }  //右边线
        void EightRegionScan_2_NC(Margin Line, uint8_t ScanCnt) // 第二路段，八邻域扫描，左，基本操作
        {
            //八邻域 边界追踪
            uint8_t i = 0;
            if (Record_Flag == 0 && Line.row >= 60)
            {
                if (LeftLine_2.Rec_Point == 0)
                {
                    LeftLine_2.Rec_Point = (byte)(ScanCnt == 0 ? 1 : ScanCnt);
                }
                SetText_1("Rec Line.row = " + Line.row + " Line.line = " + Line.line+ " LeftLine_2.Rec_Point = "+ LeftLine_2.Rec_Point);
                Record_Flag = 1;
            }
            for (i = 0; i < 8; i++)
            {
                switch ((Line.direction + i) % 8)
                {
                    case 0://右下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line - 1] == black)
                            {
                                LeftMargin_2[ScanCnt + 1].row = (byte)(Line.row - 1);
                                LeftMargin_2[ScanCnt + 1].line = (byte)(Line.line - 1);
                                LeftMargin_2[ScanCnt + 1].direction = (byte)NC_Direction.NC_downleft;
                                Mask1[LeftMargin_2[ScanCnt + 1].row, LeftMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 1://右边的点
                        {
                            if (J_Pixels[Line.row][Line.line - 1] == black)
                            {
                                LeftMargin_2[ScanCnt + 1].row = (byte)(Line.row);
                                LeftMargin_2[ScanCnt + 1].line = (byte)(Line.line - 1);
                                LeftMargin_2[ScanCnt + 1].direction = (byte)NC_Direction.NC_downmiddle;
                                Mask1[LeftMargin_2[ScanCnt + 1].row, LeftMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                if (Record_Flag != 1 && my_fabs(Line.row-LeftMargin_2[0].row) > 5)
                                    LeftLine_2.Turn_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 2://右上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line - 1] == black)
                            {
                                LeftMargin_2[ScanCnt + 1].row = (byte)(Line.row + 1);
                                LeftMargin_2[ScanCnt + 1].line = (byte)(Line.line - 1);
                                LeftMargin_2[ScanCnt + 1].direction = (byte)NC_Direction.NC_downright;
                                Mask1[LeftMargin_2[ScanCnt + 1].row, LeftMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                //记录赛道信息
                                if (Record_Flag != 1)
                                    LeftLine_2.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 3://正上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line] == black)
                            {
                                LeftMargin_2[ScanCnt + 1].row = (byte)(Line.row + 1);
                                LeftMargin_2[ScanCnt + 1].line = (byte)(Line.line);
                                LeftMargin_2[ScanCnt + 1].direction = (byte)NC_Direction.NC_equalright;
                                Mask1[LeftMargin_2[ScanCnt + 1].row, LeftMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                //记录赛道信息
                                if (Record_Flag != 1)
                                    LeftLine_2.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 4://左上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line + 1] == black)
                            {
                                LeftMargin_2[ScanCnt + 1].row = (byte)(Line.row + 1);
                                LeftMargin_2[ScanCnt + 1].line = (byte)(Line.line + 1);
                                LeftMargin_2[ScanCnt + 1].direction = (byte)NC_Direction.NC_upright;
                                Mask1[LeftMargin_2[ScanCnt + 1].row, LeftMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 5://正左边的点
                        {
                            if (J_Pixels[Line.row][Line.line + 1] == black)
                            {
                                LeftMargin_2[ScanCnt + 1].row = (byte)(Line.row);
                                LeftMargin_2[ScanCnt + 1].line = (byte)(Line.line + 1);
                                LeftMargin_2[ScanCnt + 1].direction = (byte)NC_Direction.NC_upmiddle;
                                Mask1[LeftMargin_2[ScanCnt + 1].row, LeftMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                //记录赛道信息
                                if (Record_Flag != 1)
                                    LeftLine_2.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 6://左下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line + 1] == black)
                            {
                                LeftMargin_2[ScanCnt + 1].row = (byte)(Line.row - 1);
                                LeftMargin_2[ScanCnt + 1].line = (byte)(Line.line + 1);
                                LeftMargin_2[ScanCnt + 1].direction = (byte)NC_Direction.NC_upleft;
                                Mask1[LeftMargin_2[ScanCnt + 1].row, LeftMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                //记录赛道信息
                                if (Record_Flag != 1)
                                    LeftLine_2.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 7://正下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line] == black)
                            {
                                LeftMargin_2[ScanCnt + 1].row = (byte)(Line.row - 1);
                                LeftMargin_2[ScanCnt + 1].line = (byte)(Line.line);
                                LeftMargin_2[ScanCnt + 1].direction = (byte)NC_Direction.NC_equalleft;
                                Mask1[LeftMargin_2[ScanCnt + 1].row, LeftMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                //记录赛道信息
                                if (Record_Flag != 1)
                                    LeftLine_2.Reserve_Cnt++;
                                return;
                            }
                            break;
                        }
                }
            }
        }
        void EightRegionScan_2_SC(Margin Line, uint8_t ScanCnt) // 第二路段，八邻域扫描，右，基本操作
        {
            //八邻域 边界追踪
            uint8_t i = 0;
            if (Record_Flag == 0 && Line.row >= 60)
            {
                if (RightLine_2.Rec_Point == 0)
                    RightLine_2.Rec_Point = (byte)(ScanCnt == 0 ? 1 : ScanCnt);;
                SetText_1("Rec Line.row = " + Line.row + " Line.line = " + Line.line+ " RightLine_2.Rec_Point = "+ RightLine_2.Rec_Point);
                Record_Flag = 1;
            }
            for (i = 0; i < 8; i++)
            {
                switch ((Line.direction + i) % 8)
                {
                    case 0://左下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line + 1] == black)
                            {
                                RightMargin_2[ScanCnt + 1].row = (byte)(Line.row - 1);
                                RightMargin_2[ScanCnt + 1].line = (byte)(Line.line + 1);
                                RightMargin_2[ScanCnt + 1].direction = (byte)SC_Direction.SC_downright;
                                Mask2[RightMargin_2[ScanCnt + 1].row, RightMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 1://左边的点
                        {
                            if (J_Pixels[Line.row][Line.line + 1] == black)
                            {
                                RightMargin_2[ScanCnt + 1].row = (byte)(Line.row);
                                RightMargin_2[ScanCnt + 1].line = (byte)(Line.line + 1);
                                RightMargin_2[ScanCnt + 1].direction = (byte)SC_Direction.SC_downmiddle;
                                Mask2[RightMargin_2[ScanCnt + 1].row, RightMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                if (Record_Flag != 1 && my_fabs(Line.row- RightMargin_2[0].row) > 5)
                                    RightLine_2.Turn_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 2://左上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line + 1] == black)
                            {
                                RightMargin_2[ScanCnt + 1].row = (byte)(Line.row + 1);
                                RightMargin_2[ScanCnt + 1].line = (byte)(Line.line + 1);
                                RightMargin_2[ScanCnt + 1].direction = (byte)SC_Direction.SC_downleft;
                                Mask2[RightMargin_2[ScanCnt + 1].row, RightMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                if (Record_Flag != 1)                                            //记录赛道信息
                                    RightLine_2.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 3://正上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line] == black)
                            {
                                RightMargin_2[ScanCnt + 1].row = (byte)(Line.row + 1);
                                RightMargin_2[ScanCnt + 1].line = (byte)(Line.line);
                                RightMargin_2[ScanCnt + 1].direction = (byte)SC_Direction.SC_equalleft;
                                Mask2[RightMargin_2[ScanCnt + 1].row, RightMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                if (Record_Flag != 1)                                                          //记录赛道信息
                                    RightLine_2.Straight_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 4://右上角的点
                        {
                            if (J_Pixels[Line.row + 1][Line.line - 1] == black)
                            {
                                RightMargin_2[ScanCnt + 1].row = (byte)(Line.row + 1);
                                RightMargin_2[ScanCnt + 1].line = (byte)(Line.line - 1);
                                RightMargin_2[ScanCnt + 1].direction = (byte)SC_Direction.SC_upleft;
                                Mask2[RightMargin_2[ScanCnt + 1].row, RightMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                return;
                            }
                            break;
                        }
                    case 5://正右边的点
                        {
                            if (J_Pixels[Line.row][Line.line - 1] == black)
                            {
                                RightMargin_2[ScanCnt + 1].row = (byte)(Line.row);
                                RightMargin_2[ScanCnt + 1].line = (byte)(Line.line - 1);
                                RightMargin_2[ScanCnt + 1].direction = (byte)SC_Direction.SC_upmiddle;
                                Mask2[RightMargin_2[ScanCnt + 1].row, RightMargin_2[ScanCnt + 1].line] = 1;//在图像中标记
                                if (Record_Flag != 1)                                                      //记录赛道信息
                                    RightLine_2.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 6://右下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line - 1] == black)
                            {
                                RightMargin_2[ScanCnt + 1].row = (byte)(Line.row - 1);
                                RightMargin_2[ScanCnt + 1].line = (byte)(Line.line - 1);
                                RightMargin_2[ScanCnt + 1].direction = (byte)SC_Direction.SC_upright;
                                Mask2[RightMargin_2[ScanCnt + 1].row, RightMargin_2[ScanCnt + 1].line] = 1;//在图像中标记   
                                if (Record_Flag != 1)                                     //记录赛道信息
                                    RightLine_2.BroadWire_Cnt++;
                                return;
                            }
                            break;
                        }
                    case 7://正下角的点
                        {
                            if (J_Pixels[Line.row - 1][Line.line] == black)
                            {
                                RightMargin_2[ScanCnt + 1].row = (byte)(Line.row - 1);
                                RightMargin_2[ScanCnt + 1].line = (byte)(Line.line);
                                RightMargin_2[ScanCnt + 1].direction = (byte)SC_Direction.SC_equalright;
                                Mask2[RightMargin_2[ScanCnt + 1].row, RightMargin_2[ScanCnt + 1].line] = 1;//在图像中标记       
                                if (Record_Flag != 1)//记录赛道信息
                                    RightLine_2.Reserve_Cnt++;
                                return;
                            }
                            break;
                        }
                }
            }
        }
        void EightRegionScanLine_1(Scan_LineType Line_RorL, byte StartPoint, byte TotalPoint) // 第一路段八邻域扫描
        {
            byte Trans_Point = StartPoint;
            byte ReFind_Flag = 0;
            Record_Flag = 0;
            SetText_1("//Scan Type_1: " + Line_RorL + " StartPoint: " + Trans_Point + " TotalPoint: " + TotalPoint);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (LeftMargin_1[0].row == 0 )
                {
                    SetText_1("! No Seed .Skip Find");
                    return;
                }
                //SetText_1("LeftMargin_1[0].row = "+ LeftMargin_1[0].row + " LeftMargin_1[0].line = "+ LeftMargin_1[0].line);
                if (LeftLine_1.EndLine == 0)
                    LeftLine_1.EndLine = LeftMargin_1[0].row;
                while (Trans_Point < TotalPoint)
                {
                    //SetText_1("LeftMargin_1["+ Trans_Point+"].row = "+ LeftMargin_1[Trans_Point].row);
                    EightRegionScan_1_NC(LeftMargin_1[Trans_Point], Trans_Point);
                    Trans_Point++;
                    if (LeftMargin_1[Trans_Point].row == 69 || LeftMargin_1[Trans_Point].line == 185 || LeftMargin_1[Trans_Point].line == 0 || LeftMargin_1[Trans_Point].row == 0
                        || (LeftMargin_1[Trans_Point].row == LeftMargin_1[0].row && LeftMargin_1[Trans_Point].line == LeftMargin_1[0].line)
                        || Reflect_Detect(Scan_LineType.LeftType, RoadNum.RoadNum_1, Trans_Point) == 1)
                    {
                        if (ReFind_Flag == 0 && Trans_Point <= 10)
                        {
                            SetText_1("Seed ReFind");
                            if (NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftMargin_1[Trans_Point - 1].line>10?LeftMargin_1[Trans_Point - 1].line - 10:1), (byte)(LeftMargin_1[Trans_Point - 1].row + 1), (byte)(LeftMargin_1[Trans_Point - 1].row + 20)) == 0)
                            {
                                LeftMargin_1[0].row = LeftMargin_1[0].line = 0;
                                Trans_Point = 0;
                                break;
                            }
                            ReFind_Flag = 1;
                            Trans_Point = 0;
                        }
                        else if (Trans_Point > 5)
                        {
                            break;
                        }
                        else if (ReFind_Flag == 1)
                        {
                            if (Trans_Point > 5)
                                break;
                            else
                            {
                                LeftMargin_1[0].row = 0;
                                Trans_Point = 0;
                                LeftLine_1.Error = 1;
                                LeftLine_1.StartLine = LeftLine_1.EndLine = 0;
                                SetText_1("Left Line 1 Seed Error Twice !");
                                break;
                            }
                        }
                    }
                    //SetText_1("LeftMargin_1["+Trans_Point+"].Row = " + LeftMargin_1[Trans_Point].row+ " LeftMargin_1["+Trans_Point+"].Line = "+ LeftMargin_1[Trans_Point].line);
                    if (LeftMargin_1[Trans_Point].row > LeftLine_1.EndLine)
                        LeftLine_1.EndLine = LeftMargin_1[Trans_Point].row;
                }
                if (LeftLine_1.Rec_Point == 0)
                    LeftLine_1.Rec_Point = Trans_Point;
                LeftLine_1.PointCnt = Trans_Point;//记录点的个数  可判断赛道类型等
                //SetText_1("LeftLine_1.EndLine = " + LeftLine_1.EndLine);
            }
            else
            {
                if (RightMargin_1[0].row == 0 )
                {
                    SetText_1("! No Seed .Skip Find");
                    return;
                }
                if (RightLine_1.EndLine == 0)
                    RightLine_1.EndLine = RightMargin_1[0].row;
                while (Trans_Point < TotalPoint)
                {
                    EightRegionScan_1_SC(RightMargin_1[Trans_Point], Trans_Point);
                    Trans_Point++;
                    //SetText_1("RightMargin_1[" + Trans_Point+"].Row = " + RightMargin_1[Trans_Point].row+ " RightMargin_1[" + Trans_Point+"].Line = "+ RightMargin_1[Trans_Point].line);
                    if (RightMargin_1[Trans_Point].row == 69 || RightMargin_1[Trans_Point].line == 185 || RightMargin_1[Trans_Point].line == 0 || RightMargin_1[Trans_Point].row == 0
                        || (RightMargin_1[Trans_Point].row == RightMargin_1[0].row && RightMargin_1[Trans_Point].line == RightMargin_1[0].line)
                        || Reflect_Detect(Scan_LineType.RightType, RoadNum.RoadNum_1, Trans_Point) == 1)
                    {
                        if (ReFind_Flag == 0&& Trans_Point <= 10 )
                        {
                            SetText_1("Seed ReFind");
                            if (NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(RightMargin_1[Trans_Point - 1].line<174?RightMargin_1[Trans_Point - 1].line + 10:184), (byte)(RightMargin_1[Trans_Point - 1].row + 1), (byte)(RightMargin_1[Trans_Point - 1].row + 20)) == 0)
                            {
                                RightMargin_1[0].row = RightMargin_1[0].line = 0;
                                Trans_Point = 0;
                                break;
                            }                            
                            ReFind_Flag = 1;
                            Trans_Point = 0;
                        }
                        else if (Trans_Point > 5)
                        {
                            break;
                        }
                        else if(ReFind_Flag==1)
                        {
                            if (Trans_Point > 5)
                                break;
                            else
                            {
                                RightMargin_1[0].row = 0;
                                RightLine_1.Error = 1;
                                RightLine_1.StartLine = RightLine_1.EndLine = 0;
                                Trans_Point = 0;
                                SetText_1("Right Line 1 Seed Error Twice !");
                                break;
                            }
                        }
                    }
                    if (RightLine_1.EndLine < RightMargin_1[Trans_Point].row)
                        RightLine_1.EndLine = RightMargin_1[Trans_Point].row;
                }
                if (RightLine_1.Rec_Point == 0)
                    RightLine_1.Rec_Point = Trans_Point;
                RightLine_1.PointCnt = Trans_Point;
                //SetText_1("RightLine_1.EndLine = " + RightLine_1.EndLine);
            }
            SetText_1("EndPoint " + Trans_Point);
        }
        void EightRegionScanLine_2(Scan_LineType Line_RorL, byte StartPoint, byte TotalPoint) // 第二路段八邻域扫描
        {
            byte Trans_Point = StartPoint;
            Record_Flag = 0;
            byte ReFind_Flag = 0;
            SetText_1("//Scan Type_2 : " + Line_RorL + " StartPoint: " + Trans_Point + " TotalPoint: " + TotalPoint);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (LeftMargin_2[0].row == 0 )
                {
                    SetText_1("! No Seed .Skip Find");
                    return;
                }
                if (LeftLine_2.EndLine == 0)
                    LeftLine_2.EndLine = LeftMargin_2[0].row;
                while (Trans_Point < TotalPoint)
                {
                    EightRegionScan_2_NC(LeftMargin_2[Trans_Point], Trans_Point);
                    Trans_Point++;
                    //SetText_1("LeftMargin_2["+Trans_Point+"] = "+ LeftMargin_2[Trans_Point].row + " " + LeftMargin_2[Trans_Point].line);
                    if (LeftMargin_2[Trans_Point].row == 69 || LeftMargin_2[Trans_Point].line == 185 || LeftMargin_2[Trans_Point].line == 0 || LeftMargin_2[Trans_Point].row == 0
                        || (LeftMargin_2[Trans_Point].row == LeftMargin_2[0].row && LeftMargin_2[Trans_Point].line == LeftMargin_2[0].line)
                        || Reflect_Detect(Scan_LineType.LeftType, RoadNum.RoadNum_2, Trans_Point) == 1)
                    {
                        if (ReFind_Flag == 0 && Trans_Point <= 10)
                        {
                            SetText_1("Seed 2 ReFind");
                            if (NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftMargin_2[Trans_Point - 1].line>10?LeftMargin_2[Trans_Point - 1].line - 10: 1), (byte)(LeftMargin_2[Trans_Point - 1].row + 1), 65) == 0)
                            {
                                LeftMargin_2[0].row = LeftMargin_2[0].line = 0;
                                Trans_Point = 0;
                                break;
                            }
                            ReFind_Flag = 1;
                            Trans_Point = 0;
                        }
                        else if (Trans_Point > 5)
                        {
                            break;
                        }
                        else if (ReFind_Flag == 1)
                        {
                            if (Trans_Point > 5)
                                break;
                            else
                            {
                                LeftMargin_2[0].row = 0;
                                LeftLine_2.Error = 1;
                                LeftLine_2.StartLine = LeftLine_2.EndLine = 0;
                                SetText_1("Left Line 2 Seed Error Twice !");
                                Trans_Point = 0;
                                break;
                            }
                        }
                    }
                    if (LeftLine_2.EndLine < LeftMargin_2[Trans_Point].row)
                        LeftLine_2.EndLine = LeftMargin_2[Trans_Point].row;
                }
                if (LeftLine_2.Rec_Point == 0)
                    LeftLine_2.Rec_Point = Trans_Point;
                LeftLine_2.PointCnt = Trans_Point;//记录点的个数  可判断赛道类型等
                //SetText_1("LeftLine_2.EndLine = " + LeftLine_2.EndLine);
            }
            else
            {
                if (RightMargin_2[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Find");
                    return;
                }
                if (RightLine_2.EndLine == 0)
                    RightLine_2.EndLine = RightMargin_2[0].row;
                while (Trans_Point < TotalPoint)
                {
                    EightRegionScan_2_SC(RightMargin_2[Trans_Point], Trans_Point);
                    Trans_Point++;
                    if (RightMargin_2[Trans_Point].row == 69 || RightMargin_2[Trans_Point].line == 185 || RightMargin_2[Trans_Point].line == 0 || RightMargin_2[Trans_Point].row == 0
                        || (RightMargin_2[Trans_Point].row == RightMargin_2[0].row && RightMargin_2[Trans_Point].line == RightMargin_2[0].line)
                        || Reflect_Detect(Scan_LineType.RightType, RoadNum.RoadNum_2, Trans_Point) == 1)
                    {
                        //SetText_1("RightMargin_2["+Trans_Point+"].row = "+ RightMargin_2[Trans_Point].row+ " RightMargin_2["+Trans_Point+"].line =  "+ RightMargin_2[Trans_Point].line);
                        if (ReFind_Flag == 0 && Trans_Point <= 10)
                        {
                            SetText_1("Seed 2 ReFind");
                            if (NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(RightMargin_2[Trans_Point - 1].line<174?RightMargin_2[Trans_Point - 1].line + 10: 184), (byte)(RightMargin_2[Trans_Point - 1].row + 1), 65) == 0)
                            {
                                RightMargin_2[0].row = RightMargin_2[0].line = 0;
                                Trans_Point = 0;
                                break;
                            }
                            ReFind_Flag = 1;
                            Trans_Point = 0;
                        }
                        else if (Trans_Point > 5)
                        {
                            break;
                        }
                        else if (ReFind_Flag == 1)
                        {
                            if (Trans_Point > 5)
                                break;
                            else
                            {
                                RightMargin_2[0].row = 0;
                                RightLine_2.Error = 1;
                                RightLine_2.StartLine = RightLine_2.EndLine = 0;
                                Trans_Point = 0;
                                SetText_1("Right Line 2 Seed Error Twice !");
                                break;
                            }
                        }
                    }
                    //SetText_1("RightMargin_2[" + Trans_Point + "].row = " + RightMargin_2[Trans_Point].row + " RightMargin_2[" + Trans_Point + "].line =  " + RightMargin_2[Trans_Point].line);
                    if (RightLine_2.EndLine < RightMargin_2[Trans_Point].row)
                        RightLine_2.EndLine = RightMargin_2[Trans_Point].row;
                }
                if (RightLine_2.Rec_Point == 0)
                    RightLine_2.Rec_Point = Trans_Point;
                RightLine_2.PointCnt = Trans_Point;
                //SetText_1("RightLine_2.EndLine = "+ RightLine_2.EndLine);
            }
            SetText_1("EndPoint " + Trans_Point);
        }
        byte Traverse_Agl_1(Scan_LineType Line_RorL, byte EndRow) // 寻找第一路段扫描得到的邻域边线，寻找直角型拐点
        {
            //byte Trans_Point = StartPoint;
            byte Trans_Point = 1;
            SetText_1("//Traverse_Agl_1: " + Line_RorL + " StartPoint: " + Trans_Point + " EndRow = "+ EndRow);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (LeftMargin_1[0].row == 0 )
                {
                    SetText_1("! No Seed .Skip Traverse");
                    return 0;
                }
                for (; Trans_Point < LeftLine_1.PointCnt; Trans_Point++)//  这一步可以优化 第一次扫线是可以进行判断  算了不优化了
                {
                    if (LeftMargin_1[Trans_Point].row >= EndRow)
                        break;
                    if (LeftMargin_1[Trans_Point].direction == (byte)NC_Direction.NC_upmiddle)//通过判断下一个点的位置来判断
                    {
                       // if ((Trans_Point>5? MarginStrErrorCale_1((byte)(Trans_Point-5), Trans_Point, 0) : MarginStrErrorCale_1(0, Trans_Point, 0))<=2
                       //     && (Trans_Point+5<=LeftLine_1.PointCnt? MarginStrErrorCale_1(Trans_Point, (byte)(Trans_Point+5), 0) : MarginStrErrorCale_1(Trans_Point, LeftLine_1.PointCnt, 0)) <= 2)
                       // {
                            SetText_1("* LeftAngley_1:" + "   row:" + LeftMargin_1[Trans_Point - 1].row + " line" + LeftMargin_1[Trans_Point - 1].line);
                            //记录角点坐标
                            LeftLine_1.Agl_Row = LeftMargin_1[Trans_Point - 1].row;
                            LeftLine_1.Agl_Line = LeftMargin_1[Trans_Point - 1].line;
                            LeftLine_1.Agl_PointNum = (byte)(Trans_Point - 1);
                            return 1;
                       // }
                    }
                }
                SetText_1("! No LeftAngley_1");
            }
            if (Line_RorL == Scan_LineType.RightType)
            {
                if (RightMargin_1[0].row == 0 )
                {
                    SetText_1("! No Seed .Skip Traverse");
                    return 0;
                }
                for (; Trans_Point < RightLine_1.PointCnt; Trans_Point++)
                {
                    //SetText_1("RightMargin_1["+Trans_Point+"].row = "+ RightMargin_1[Trans_Point].row + " RightMargin_1["+Trans_Point+"].line = "+ RightMargin_1[Trans_Point].row);
                    if (RightMargin_1[Trans_Point].row >= EndRow)
                        break;
                    if (RightMargin_1[Trans_Point].direction == (byte)SC_Direction.SC_upmiddle)//通过判断下一个点的位置来判断
                    {
                      //  if ((Trans_Point > 5 ? MarginStrErrorCale_1((byte)(Trans_Point - 5), Trans_Point, 1) : MarginStrErrorCale_1(0, Trans_Point, 1)) <= 2
                      //      && (Trans_Point + 5 <= RightLine_1.PointCnt ? MarginStrErrorCale_1(Trans_Point, (byte)(Trans_Point + 5), 1) : MarginStrErrorCale_1(Trans_Point, RightLine_1.PointCnt, 1)) <= 2)
                       // {
                            SetText_1("* RightAngley_1:" + "   row:" + RightMargin_1[Trans_Point - 1].row + " line" + RightMargin_1[Trans_Point - 1].line);
                            RightLine_1.Agl_Row = RightMargin_1[Trans_Point - 1].row;
                            RightLine_1.Agl_Line = RightMargin_1[Trans_Point - 1].line;
                            RightLine_1.Agl_PointNum = (byte)(Trans_Point - 1);
                            return 1;
                      //  }
                    }
                }
                SetText_1("! No RightAngley_1");
            }
            return 0;
        }
        byte Traverse_Agl_2(Scan_LineType Line_RorL, byte EndRow)// 寻找第二路段扫描得到的邻域边线，寻找直角型拐点
        {
            byte Trans_Point = 1;
            SetText_1("//Traverse_Agl_2: " + Line_RorL + " StartPoint: " + Trans_Point + " EndRow = " + EndRow);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (LeftMargin_2[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Traverse");
                    return 0;
                }
                for (; Trans_Point < LeftLine_2.PointCnt; Trans_Point++)//  这一步可以优化 第一次扫线是可以进行判断  算了不优化了
                {
                    if (LeftMargin_2[Trans_Point].row >= EndRow)
                        break;
                    if ((LeftMargin_2[Trans_Point].direction == (byte)NC_Direction.NC_downright || LeftMargin_2[Trans_Point].direction == (byte)NC_Direction.NC_equalright)//通过判断下一个点的位置来判断
                     && (LeftMargin_2[Trans_Point - 1].direction == (byte)NC_Direction.NC_downright || LeftMargin_2[Trans_Point - 1].direction == (byte)NC_Direction.NC_equalright)
                        )
                    {
                        SetText_1("* LeftAngley_2:" + "   row:" + LeftMargin_2[Trans_Point].row + " line" + LeftMargin_2[Trans_Point].line);
                        //记录角点坐标
                        LeftLine_2.Agl_Row = LeftMargin_2[Trans_Point].row;
                        LeftLine_2.Agl_Line = LeftMargin_2[Trans_Point].line;
                        LeftLine_2.Agl_PointNum = (byte)(Trans_Point - 1);
                        return 1;
                    }
                }
                SetText_1("! No LeftAngley_2");
            }
            else
            {
                if (RightMargin_2[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Traverse");
                    return 0;
                }
                for (; Trans_Point < RightLine_2.PointCnt; Trans_Point++)
                {
                    if (RightMargin_2[Trans_Point].row >= EndRow)
                        break;
                    if ((RightMargin_2[Trans_Point].direction == (byte)SC_Direction.SC_equalleft || RightMargin_2[Trans_Point].direction == (byte)SC_Direction.SC_downleft)
                        && (RightMargin_2[Trans_Point - 1].direction == (byte)SC_Direction.SC_equalleft || RightMargin_2[Trans_Point - 1].direction == (byte)SC_Direction.SC_downleft)
                        )
                    {
                        SetText_1("* RightAngley_2:" + "   row:" + RightMargin_2[Trans_Point].row + " line" + RightMargin_2[Trans_Point].line);
                        RightLine_2.Agl_Row = RightMargin_2[Trans_Point].row;
                        RightLine_2.Agl_Line = RightMargin_2[Trans_Point].line;
                        RightLine_2.Agl_PointNum = (byte)(Trans_Point - 1);
                        return 1;
                    }
                }
                SetText_1("! No RightAngley_2");
            }
            return 0;
        }
        byte Traverse_Circle_Agl_1(Scan_LineType Line_RorL, byte EndRow) // 寻找第一路段扫描得到的邻域边线，寻找圆弧形拐点
        {
            byte Trans_Point = 1;
            byte flag = 0;
            SetText_1("//Traverse_Circle_Agl_1: " + Line_RorL + " StartPoint: " + Trans_Point);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (LeftMargin_1[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Circle Agl Traverse");
                    return 0;
                }
                for (; Trans_Point < (LeftLine_1.PointCnt - 3); Trans_Point++)
                {
                    if (LeftMargin_1[Trans_Point].row >= EndRow)
                        return 0;
                    if (LeftMargin_1[Trans_Point].line > LeftMargin_1[Trans_Point - 1].line
                        && LeftMargin_1[Trans_Point + 1].line >= LeftMargin_1[Trans_Point].line
                        && LeftMargin_1[Trans_Point + 2].line >= LeftMargin_1[Trans_Point + 1].line
                        && LeftMargin_1[Trans_Point + 3].line >= LeftMargin_1[Trans_Point + 2].line)//通过判断下一个点的位置来判断
                    {
                        /*if (flag == 0)
                        {
                            SetText_1("! No Circle_LeftAngley_1");
                            return 0;
                        }*/
                        //if ((Trans_Point > 5 ? MarginStrErrorCale_1((byte)(Trans_Point - 5), Trans_Point, 0) : MarginStrErrorCale_1(0, Trans_Point, 0)) <= 2
                         //   && (Trans_Point + 5 <= LeftLine_1.PointCnt ? MarginStrErrorCale_1(Trans_Point, (byte)(Trans_Point + 5), 0) : MarginStrErrorCale_1(Trans_Point, LeftLine_1.PointCnt, 0)) <= 2)
                        //{
                            SetText_1("* Circle_LeftAngley_1:" + "   row:" + LeftMargin_1[Trans_Point - 1].row + " line" + LeftMargin_1[Trans_Point - 1].line);
                            //记录角点坐标
                            LeftLine_1.Agl_Row = LeftMargin_1[Trans_Point - 1].row;
                            LeftLine_1.Agl_Line = LeftMargin_1[Trans_Point - 1].line;
                            LeftLine_1.Agl_PointNum = (byte)(Trans_Point - 1);
                            return 1;
                       // }
                    }
                   /* else if (LeftMargin_1[Trans_Point].line <= LeftMargin_1[Trans_Point + 1].line
                        && LeftMargin_1[Trans_Point+1].line <= LeftMargin_1[Trans_Point + 2].line
                        && LeftMargin_1[Trans_Point+2].line <= LeftMargin_1[Trans_Point + 3].line)
                        flag = 1;*/
                    //SetText_1("LeftMargin_1[" + Trans_Point + "].row = " + LeftMargin_1[Trans_Point].row + " LeftMargin_1[" + Trans_Point + "].line =  " + LeftMargin_1[Trans_Point].line);

                }
                SetText_1("! No Circle_LeftAngley_1");
            }
            if (Line_RorL == Scan_LineType.RightType)
            {
                if (RightMargin_1[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Circle Agl Traverse");
                    return 0;
                }
                for (; Trans_Point < (byte)(RightLine_1.PointCnt - 3); Trans_Point++)
                {
                    if (RightMargin_1[Trans_Point].row >= EndRow)
                        return 0;
                    if (RightMargin_1[Trans_Point].line < RightMargin_1[Trans_Point - 1].line
                        && RightMargin_1[Trans_Point + 1].line <= RightMargin_1[Trans_Point].line
                        && RightMargin_1[Trans_Point + 2].line <= RightMargin_1[Trans_Point + 1].line
                        && RightMargin_1[Trans_Point + 3].line <= RightMargin_1[Trans_Point + 2].line)//通过判断下一个点的位置来判断
                    {
                        /*if (flag == 0)
                        {
                            SetText_1("! No Circle_RightAngley_1");
                            return 0;
                        }*/
                        //if ((Trans_Point > 5 ? MarginStrErrorCale_1((byte)(Trans_Point - 5), Trans_Point, 1) : MarginStrErrorCale_1(0, Trans_Point, 1)) <= 2
                        //    && (Trans_Point + 5 <= RightLine_1.PointCnt ? MarginStrErrorCale_1(Trans_Point, (byte)(Trans_Point + 5), 1) : MarginStrErrorCale_1(Trans_Point, RightLine_1.PointCnt, 1)) <= 2)
                        //{
                            SetText_1("* Circle_RightAngley_1:" + "   row:" + RightMargin_1[Trans_Point - 1].row + " line" + RightMargin_1[Trans_Point - 1].line);
                            RightLine_1.Agl_Row = RightMargin_1[Trans_Point - 1].row;
                            RightLine_1.Agl_Line = RightMargin_1[Trans_Point - 1].line;
                            RightLine_1.Agl_PointNum = (byte)(Trans_Point - 1);
                            return 1;
                        //}
                    }
                    /*else if (RightMargin_1[Trans_Point].line > RightMargin_1[Trans_Point + 1].line
                        && RightMargin_1[Trans_Point+1].line > RightMargin_1[Trans_Point + 2].line
                        && RightMargin_1[Trans_Point+2].line > RightMargin_1[Trans_Point + 3].line)
                        flag = 1;*/
                }
                SetText_1("! No Circle_RightAngley_1");
            }
            return 0;
        }
        byte Traverse_Circle_Agl_2(Scan_LineType Line_RorL, byte EndRow) // 寻找第二路段扫描得到的邻域边线，寻找圆弧形拐点
        {
            byte Trans_Point = 1;
            byte flag = 0;
            SetText_1("//Traverse_Circle_Agl_2: " + Line_RorL + " StartPoint: " + Trans_Point);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (LeftMargin_2[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Circle Agl Traverse");
                    return 0;
                }
                for (; Trans_Point < (LeftLine_2.PointCnt - 3); Trans_Point++)
                {
                    if (LeftMargin_2[Trans_Point].row >= EndRow)
                        return 0;
                    if (LeftMargin_2[Trans_Point].line > LeftMargin_2[Trans_Point - 1].line
                        && LeftMargin_2[Trans_Point + 1].line >= LeftMargin_2[Trans_Point].line
                        && LeftMargin_2[Trans_Point + 2].line >= LeftMargin_2[Trans_Point + 1].line
                        && LeftMargin_2[Trans_Point + 3].line >= LeftMargin_2[Trans_Point + 2].line)//通过判断下一个点的位置来判断
                    {
                        /*if (flag == 0)
                        {
                            SetText_1("! No Circle_LeftAngley_2");
                            return 0;
                        }*/
                       // if ((Trans_Point > 5 ? MarginStrErrorCale_2((byte)(Trans_Point - 5), Trans_Point, 0) : MarginStrErrorCale_2(0, Trans_Point, 0)) <= 2
                       //     && (Trans_Point + 5 <= LeftLine_2.PointCnt ? MarginStrErrorCale_2(Trans_Point, (byte)(Trans_Point + 5), 0) : MarginStrErrorCale_2(Trans_Point, LeftLine_2.PointCnt, 0)) <= 2)
                       // {
                            SetText_1("* Circle_LeftAngley_2:" + "   row:" + LeftMargin_2[Trans_Point - 1].row + " line" + LeftMargin_2[Trans_Point - 1].line);
                            //记录角点坐标
                            LeftLine_2.Agl_Row = LeftMargin_2[Trans_Point - 1].row;
                            LeftLine_2.Agl_Line = LeftMargin_2[Trans_Point - 1].line;
                            LeftLine_2.Agl_PointNum = (byte)(Trans_Point - 1);
                            return 1;
                      //  }
                    }
                    /*else if (LeftMargin_2[Trans_Point].line < LeftMargin_2[Trans_Point - 1].line)
                        flag = 1;*/

                }
                SetText_1("! No Circle_LeftAngley_2");
            }
            if (Line_RorL == Scan_LineType.RightType)
            {
                if (RightMargin_2[0].row == 0 )
                {
                    SetText_1("! No Seed .Skip Circle Agl Traverse");
                    return 0;
                }
                for (; Trans_Point < (byte)(RightLine_2.PointCnt - 3); Trans_Point++)
                {
                    if (RightMargin_2[Trans_Point].row >= EndRow)
                        return 0;
                    if (RightMargin_2[Trans_Point].line < RightMargin_2[Trans_Point - 1].line
                        && RightMargin_2[Trans_Point + 1].line <= RightMargin_2[Trans_Point].line
                        && RightMargin_2[Trans_Point + 2].line <= RightMargin_2[Trans_Point + 1].line
                        && RightMargin_2[Trans_Point + 3].line <= RightMargin_2[Trans_Point + 2].line)//通过判断下一个点的位置来判断
                    {
                        /*if (flag == 0)
                        {
                            SetText_1("! No Circle_RightAngley_2");
                            return 0;
                        }*/
                        //if ((Trans_Point > 5 ? MarginStrErrorCale_2((byte)(Trans_Point - 5), Trans_Point, 1) : MarginStrErrorCale_2(0, Trans_Point, 1)) <= 2
                        //    && (Trans_Point + 5 <= RightLine_2.PointCnt ? MarginStrErrorCale_2(Trans_Point, (byte)(Trans_Point + 5), 1) : MarginStrErrorCale_2(Trans_Point, RightLine_2.PointCnt, 1)) <= 2)
                        //{
                            SetText_1("* Circle_RightAngley_2:" + "   row:" + RightMargin_2[Trans_Point - 1].row + " line" + RightMargin_2[Trans_Point - 1].line);
                            RightLine_2.Agl_Row = RightMargin_2[Trans_Point - 1].row;
                            RightLine_2.Agl_Line = RightMargin_2[Trans_Point - 1].line;
                            RightLine_2.Agl_PointNum = (byte)(Trans_Point - 1);
                            return 1;
                        //}
                    }
                   /* else if (RightMargin_2[Trans_Point].line > RightMargin_2[Trans_Point - 1].line)
                        flag = 1;*/
                }
                SetText_1("! No Circle_RightAngley_2");
            }
            return 0;
        }
        byte Traverse_Loose_Agl_2(Scan_LineType Line_RorL, byte EndRow) // 寻找第二路段扫描得到的邻域边线，寻找圆弧形拐点，条件更松
        {
            byte Trans_Point = 1;
            SetText_1("//Traverse Loose Agl_2: " + Line_RorL + " StartPoint: " + Trans_Point + " EndRow = " + EndRow);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (LeftMargin_2[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Loose Traverse");
                    return 0;
                }
                for (; Trans_Point < LeftLine_2.PointCnt; Trans_Point++)//  这一步可以优化 第一次扫线是可以进行判断  算了不优化了
                {
                    if (LeftMargin_2[Trans_Point].row >= EndRow)
                        break;
                    if (LeftMargin_2[Trans_Point].direction == (byte)NC_Direction.NC_downright || LeftMargin_2[Trans_Point].direction == (byte)NC_Direction.NC_equalright)
                    {
                        SetText_1("* Loose LeftAngley_2:" + "   row:" + LeftMargin_2[Trans_Point].row + " line" + LeftMargin_2[Trans_Point].line);
                        //记录角点坐标
                        LeftLine_2.Agl_Row = LeftMargin_2[Trans_Point].row;
                        LeftLine_2.Agl_Line = LeftMargin_2[Trans_Point].line;
                        LeftLine_2.Agl_PointNum = (byte)(Trans_Point - 1);
                        return 1;
                    }
                }
                SetText_1("! No Loose LeftAngley_2");
            }
            else
            {
                if (RightMargin_2[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Loose Traverse");
                    return 0;
                }
                for (; Trans_Point < RightLine_2.PointCnt; Trans_Point++)
                {
                    if (RightMargin_2[Trans_Point].row >= EndRow)
                        break;
                    if (RightMargin_2[Trans_Point].direction == (byte)SC_Direction.SC_equalleft || RightMargin_2[Trans_Point].direction == (byte)SC_Direction.SC_downleft)
                    {
                        SetText_1("* Loose RightAngley_2:" + "   row:" + RightMargin_2[Trans_Point].row + " line" + RightMargin_2[Trans_Point].line);
                        RightLine_2.Agl_Row = RightMargin_2[Trans_Point].row;
                        RightLine_2.Agl_Line = RightMargin_2[Trans_Point].line;
                        RightLine_2.Agl_PointNum = (byte)(Trans_Point - 1);
                        return 1;
                    }
                }
                SetText_1("! No Loose RightAngley_2");
            }
            return 0;
        }
        byte Traverse_Turn_Agl_1(Scan_LineType Line_RorL, byte EndRow) // // 寻找第二路段扫描得到的邻域边线，寻找弯道拐点
        {
            int Trans_Point = 1;
            byte flag = 0;
            byte Record_flag = 0;
            SetText_1("//Traverse_Circle_Agl_1: " + Line_RorL + " StartPoint: " + Trans_Point);
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (LeftMargin_1[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Circle Agl Traverse");
                    return 0;
                }
                for (; Trans_Point < (LeftLine_1.PointCnt - 3); Trans_Point++)
                {
                    if (LeftMargin_1[Trans_Point].row >= EndRow || LeftMargin_1[Trans_Point].row == 0)
                        return 0;
                    if (LeftMargin_1[Trans_Point].line > LeftMargin_1[Trans_Point - 1].line
                        && LeftMargin_1[Trans_Point + 1].line >= LeftMargin_1[Trans_Point].line
                        && LeftMargin_1[Trans_Point + 2].line >= LeftMargin_1[Trans_Point].line
                        && LeftMargin_1[Trans_Point + 3].line >= LeftMargin_1[Trans_Point].line)//通过判断下一个点的位置来判断
                    {
                        /*if (flag == 0)
                        {
                            SetText_1("! No Circle_LeftAngley_1");
                            return 0;
                        }*/
                        if (Record_flag == 0)
                        {
                            SetText_1("* Circle_LeftAngley_1:" + "   row:" + LeftMargin_1[Trans_Point - 1].row + " line" + LeftMargin_1[Trans_Point - 1].line);
                            //记录角点坐标
                            LeftLine_1.Agl_Row = LeftMargin_1[Trans_Point - 1].row;
                            LeftLine_1.Agl_Line = LeftMargin_1[Trans_Point - 1].line;
                            LeftLine_1.Agl_PointNum = (byte)(Trans_Point - 1);
                            Record_flag++;
                        }
                        else if (flag == 1&&Record_flag == 1 && LeftMargin_1[Trans_Point].row > LeftLine_1.Agl_Row + 5)
                        {
                            SetText_1("* Circle_LeftAngley_2:" + "   row:" + LeftMargin_1[Trans_Point - 1].row + " line" + LeftMargin_1[Trans_Point - 1].line);
                            //记录角点坐标
                            LeftLine_1.Agl_2_Row = LeftMargin_1[Trans_Point - 1].row;
                            LeftLine_1.Agl_2_Line = LeftMargin_1[Trans_Point - 1].line;
                            Record_flag++;
                            return 1;
                        }
                    }
                    else if (Record_flag == 1&&LeftMargin_1[Trans_Point].line < LeftMargin_1[Trans_Point - 1].line)
                        flag = 1;
                    //SetText_1("LeftMargin_1[" + Trans_Point + "].row = " + LeftMargin_1[Trans_Point].row + " LeftMargin_1[" + Trans_Point + "].line =  " + LeftMargin_1[Trans_Point].line);

                }
                SetText_1("! No Circle_LeftAngley_1");
            }
            if (Line_RorL == Scan_LineType.RightType)
            {
                if (RightMargin_1[0].row == 0)
                {
                    SetText_1("! No Seed .Skip Circle Agl Traverse");
                    return 0;
                }
                for (; Trans_Point < (byte)(RightLine_1.PointCnt - 3); Trans_Point++)
                {
                    if (RightMargin_1[Trans_Point].row >= EndRow|| RightMargin_1[Trans_Point].row==0)
                        return 0;
                    if (RightMargin_1[Trans_Point].line < RightMargin_1[Trans_Point - 1].line
                        && RightMargin_1[Trans_Point + 1].line <= RightMargin_1[Trans_Point].line
                        && RightMargin_1[Trans_Point + 2].line <= RightMargin_1[Trans_Point].line
                        && RightMargin_1[Trans_Point + 3].line <= RightMargin_1[Trans_Point].line)//通过判断下一个点的位置来判断
                    {
                        /*if (flag == 0)
                        {
                            SetText_1("! No Circle_RightAngley_1");
                            return 0;
                        }*/
                        if (Record_flag == 0)
                        {
                            SetText_1("* Circle_RightAngley_1:" + "   row:" + RightMargin_1[Trans_Point - 1].row + " line" + RightMargin_1[Trans_Point - 1].line);
                            RightLine_1.Agl_Row = RightMargin_1[Trans_Point - 1].row;
                            RightLine_1.Agl_Line = RightMargin_1[Trans_Point - 1].line;
                            RightLine_1.Agl_PointNum = (byte)(Trans_Point - 1);
                            Record_flag++;
                        }
                        else if(flag == 1&&Record_flag ==1 
                            && RightMargin_1[Trans_Point].row> RightLine_1.Agl_Row+5
                            && RightMargin_1[Trans_Point].line != RightLine_1.Agl_Line)
                        {
                            SetText_1("* Circle_RightAngley_2:" + "   row:" + RightMargin_1[Trans_Point - 1].row + " line" + RightMargin_1[Trans_Point - 1].line);
                            RightLine_1.Agl_2_Row = RightMargin_1[Trans_Point - 1].row;
                            RightLine_1.Agl_2_Line = RightMargin_1[Trans_Point - 1].line;
                            return 1;
                        }
                        
                    }
                    else if (Record_flag == 1&&RightMargin_1[Trans_Point].line > RightMargin_1[Trans_Point - 1].line)
                        flag = 1;
                }
                SetText_1("! No Circle_RightAngley_1");
            }
            return 0;
        }
        byte Scan_And_Traverse_Agl_1(Scan_LineType Line_RorL, byte Start_Point, byte ScanCnt) // 第一路段，扫描并寻找拐点
        {
            if (Line_RorL == Scan_LineType.LeftType)
            {
                EightRegionScanLine_1(Scan_LineType.LeftType, Start_Point, ScanCnt);
                return Traverse_Agl_1(Scan_LineType.LeftType, 65);
            }
            else
            {
                EightRegionScanLine_1(Scan_LineType.RightType, Start_Point, ScanCnt);
                return Traverse_Agl_1(Scan_LineType.RightType, 65);
            }
        }
        byte Scan_And_Traverse_Agl_2(Scan_LineType Line_RorL, byte Start_Point, byte ScanCnt) // 第二路段，扫描并寻找拐点
        {
            if (Line_RorL == Scan_LineType.LeftType)
            {
                EightRegionScanLine_2(Scan_LineType.LeftType, Start_Point, ScanCnt);
                return Traverse_Agl_2(Scan_LineType.LeftType, 65);
            }
            else
            {
                EightRegionScanLine_2(Scan_LineType.RightType, Start_Point, ScanCnt);
                return Traverse_Agl_2(Scan_LineType.RightType, 65);
            }
        }
        byte Scan_And_Traverse_Loose_Agl_2(Scan_LineType Line_RorL, byte Start_Point, byte ScanCnt) // 顾名思义，同上
        {
            if (Line_RorL == Scan_LineType.LeftType)
            {
                EightRegionScanLine_2(Scan_LineType.LeftType, Start_Point, ScanCnt);
                return Traverse_Loose_Agl_2(Scan_LineType.LeftType, 65);
            }
            else
            {
                EightRegionScanLine_2(Scan_LineType.RightType, Start_Point, ScanCnt);
                return Traverse_Loose_Agl_2(Scan_LineType.RightType, 65);
            }
        }
        byte Scan_And_Traverse_Circle_Agl_1(Scan_LineType Line_RorL, byte Start_Point, byte ScanCnt) // 顾名思义，同上
        {
            if (Line_RorL == Scan_LineType.LeftType)
            {
                EightRegionScanLine_1(Scan_LineType.LeftType, Start_Point, ScanCnt);
                return Traverse_Circle_Agl_1(Scan_LineType.LeftType, 65);
            }
            else
            {
                EightRegionScanLine_1(Scan_LineType.RightType, Start_Point, ScanCnt);
                return Traverse_Circle_Agl_1(Scan_LineType.RightType, 65);
            }
        }
        byte Scan_And_Traverse_Circle_Agl_2(Scan_LineType Line_RorL, byte Start_Point, byte ScanCnt) // 顾名思义，同上
        {
            if (Line_RorL == Scan_LineType.LeftType)
            {
                EightRegionScanLine_2(Scan_LineType.LeftType, Start_Point, ScanCnt);
                return Traverse_Circle_Agl_2(Scan_LineType.LeftType, 65);
            }
            else
            {
                EightRegionScanLine_2(Scan_LineType.RightType, Start_Point, ScanCnt);
                return Traverse_Circle_Agl_2(Scan_LineType.RightType, 65);
            }
        }
        #endregion
        #region 八领域最小二乘法
        void MarginReg_1(byte startPoint,byte endPoint, Scan_LineType Line_RorL)  //0为左，1为右
        {
            int i;
            float SumUp, SumDown, avrX, avrY, SumX = 0, SumY = 0, SumLines = 0;
            byte interval = 1;
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (endPoint- startPoint >= 40)
                    interval = 4;
                else if (endPoint - startPoint >= 20)
                    interval = 2;
                else
                    interval = 1;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        SumLines++;   // startpoint 为开始行， //endpoint 结束行 //SumLines
                        SumX += LeftMargin_1[i].row;
                        SumY += LeftMargin_1[i].line;
                        //SetText_1("LeftMargin_1["+i+"].row = "+ LeftMargin_1[i].row + " LeftMargin_1[i].line = "+ LeftMargin_1[i].line);
                    }
                }
                avrX = SumX / SumLines;     //X的平均值
                avrY = SumY / SumLines;     //Y的平均值       
                SumUp = 0;
                SumDown = 0;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        SumUp += ((float)LeftMargin_1[i].line - avrY) * (LeftMargin_1[i].row - avrX);
                        SumDown += (LeftMargin_1[i].row - avrX) * (LeftMargin_1[i].row - avrX);
                    }
                }
                if (SumDown == 0)
                    LeftLine_1.Line_k = 0;
                else
                    LeftLine_1.Line_k = (float)(SumUp / SumDown);
                LeftLine_1.Line_b = (SumY - LeftLine_1.Line_k * SumX) / (float)SumLines;  //截距
                //SetText_2("MarLeftLine_1.Line_k = " + LeftLine_1.Line_k + " MarLeftLine_1.Line_b = " + LeftLine_1.Line_b);
            }
            else 
            {
                if (endPoint - startPoint >= 40)
                    interval = 4;
                else if (endPoint - startPoint >= 20)
                    interval = 2;
                else
                    interval = 1;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        SumLines++;   // startpoint 为开始行， //endpoint 结束行 //SumLines
                        SumX += RightMargin_1[i].row;
                        SumY += RightMargin_1[i].line;
                        //SetText_1("RightMargin_1[" + i + "].row = " + RightMargin_1[i].row + " RightMargin_1[i].line = " + RightMargin_1[i].line);
                    }
                }
                avrX = SumX / SumLines;     //X的平均值
                avrY = SumY / SumLines;     //Y的平均值       
                SumUp = 0;
                SumDown = 0;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        SumUp += ((float)RightMargin_1[i].line - avrY) * (RightMargin_1[i].row - avrX);
                        SumDown += (RightMargin_1[i].row - avrX) * (RightMargin_1[i].row - avrX);
                    }
                }
                if (SumDown == 0)
                    RightLine_1.Line_k = 0;
                else
                    RightLine_1.Line_k = (float)(SumUp / SumDown);
                RightLine_1.Line_b = (SumY - RightLine_1.Line_k * SumX) / (float)SumLines;  //截距
                //SetText_2("MarRightLine_1.Line_k = " + RightLine_1.Line_k + " MarRightLine_1.Line_b = " + RightLine_1.Line_b);
            }
        }
        void MarginReg_2(byte startPoint, byte endPoint, Scan_LineType Line_RorL)  //0为左，1为右
        {
            int i;
            float SumUp, SumDown, avrX, avrY, SumX = 0, SumY = 0, SumLines = 0;
            byte interval = 1;
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (endPoint -startPoint >= 40)
                    interval = 4;
                else if (endPoint - startPoint >= 20)
                    interval = 2;
                else
                    interval = 1;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        SumLines++;   // startpoint 为开始行， //endpoint 结束行 //SumLines
                        SumX += LeftMargin_2[i].row;
                        SumY += LeftMargin_2[i].line;
                        //SetText_1("LeftMargin_2[" + i + "].row = " + LeftMargin_2[i].row + " LeftMargin_2[i].line = " + LeftMargin_2[i].line);
                    }
                }
                avrX = SumX / SumLines;     //X的平均值
                avrY = SumY / SumLines;     //Y的平均值       
                SumUp = 0;
                SumDown = 0;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        SumUp += ((float)LeftMargin_2[i].line - avrY) * (LeftMargin_2[i].row - avrX);
                        SumDown += (LeftMargin_2[i].row - avrX) * (LeftMargin_2[i].row - avrX);
                    }
                }
                if (SumDown == 0)
                    LeftLine_2.Line_k = 0;
                else
                    LeftLine_2.Line_k = (float)(SumUp / SumDown);
                LeftLine_2.Line_b = (SumY - LeftLine_2.Line_k * SumX) / (float)SumLines;  //截距
                //SetText_2("MarRightLine_2.Line_k = " + RightLine_2.Line_k + " MarRightLine_2.Line_b = " + RightLine_2.Line_b);
            }
            else 
            {
                if (endPoint - startPoint >= 40)
                    interval = 4;
                else if (endPoint - startPoint >= 20)
                    interval = 2;
                else
                    interval = 1;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        SumLines++;   // startpoint 为开始行， //endpoint 结束行 //SumLines
                        SumX += RightMargin_2[i].row;
                        SumY += RightMargin_2[i].line;
                        //SetText_1("LeftMargin_2[" + i + "].row = " + LeftMargin_2[i].row + " LeftMargin_2[i].line = " + LeftMargin_2[i].line);
                    }
                }
                avrX = SumX / SumLines;     //X的平均值
                avrY = SumY / SumLines;     //Y的平均值       
                SumUp = 0;
                SumDown = 0;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        SumUp += ((float)RightMargin_2[i].line - avrY) * (RightMargin_2[i].row - avrX);
                        SumDown += (RightMargin_2[i].row - avrX) * (RightMargin_2[i].row - avrX);
                    }
                }
                if (SumDown == 0)
                    RightLine_2.Line_k = 0;
                else
                    RightLine_2.Line_k = (float)(SumUp / SumDown);
                RightLine_2.Line_b = (SumY - RightLine_2.Line_k * SumX) / (float)SumLines;  //截距
                //SetText_2("MarRightLine_2.Line_k = " + RightLine_2.Line_k + " MarRightLine_1.Line_b = " + RightLine_2.Line_b);
            }
        }
        float MarginStrCmp_1(byte startPoint, byte endPoint, Scan_LineType Line_RorL)  //0为左，1为右
        {
            byte interval = 1;
            int i;
            float sum = 0;
            int regLine;
            int count = 0;
            if (Line_RorL == Scan_LineType.LeftType)
            {
                //SetText_1("End Point LeftMargin_1[" + endPoint + "] : Row = " + LeftMargin_1[endPoint].row + " Line = " + LeftMargin_1[endPoint].line);
                if (endPoint- startPoint >= 40)
                    interval = 4;
                else if (endPoint - startPoint >= 10)
                    interval = 2;
                else
                    interval = 1;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        count++;
                        regLine = (int)(LeftLine_1.Line_k * LeftMargin_1[i].row + LeftLine_1.Line_b);
                        if (regLine > 185)
                            regLine = 185;
                        else if (regLine < 0)
                            regLine = 0;
                        sum += my_fabs(LeftMargin_1[i].line - regLine);
                        //SetText_1("Left Row = "+i+" Error = " + my_fabs(LeftMargin_1[i].line - regLine) + " sum = "+sum);
                        Mask2[LeftMargin_1[i].row, regLine] = 1;
                    }
                }
                sum /= count;
                //SetText_1("LeftLine Str Error = " + sum);
                //LeftLine_1.RecStrError = sum;
            }
            else 
            {
                //SetText_1("End Point RightMargin_1["+ endPoint + "] : Row = "+ RightMargin_1[endPoint].row+ " Line = "+ RightMargin_1[endPoint].line);
                if (endPoint - startPoint >= 40)
                    interval = 4;
                else if (endPoint - startPoint >= 20)
                    interval = 2;
                else
                    interval = 1;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        count++;
                        regLine = (int)(RightLine_1.Line_k * RightMargin_1[i].row + RightLine_1.Line_b);
                        if (regLine > 185)
                            regLine = 185;
                        else if (regLine < 0)
                            regLine = 0;
                        sum += my_fabs(RightMargin_1[i].line - regLine);
                        //SetText_1("Right Row = " + i + " Error = " + my_fabs(RightMargin_1[i].line - regLine) + " sum = " + sum);
                        Mask2[RightMargin_1[i].row, regLine] = 1;
                    }
                }
                sum /= count;
                //setText用户自定义("RightLine Str Error = "+ sum);
                //RightLine_1.RecStrError = sum;
            }
            return sum;
        }
        float MarginStrCmp_2(byte startPoint, byte endPoint, Scan_LineType Line_RorL)  //0为左，1为右
        {
            byte interval = 1;
            int i;
            float sum = 0;
            int regLine;
            int count = 0;
            if (Line_RorL == Scan_LineType.LeftType)
            {
                //SetText_1("End Point LeftMargin_2[" + endPoint + "] : Row = " + LeftMargin_2[endPoint].row + " Line = " + LeftMargin_2[endPoint].line);
                if (endPoint- startPoint >= 40)
                    interval = 4;
                else if (endPoint - startPoint >= 20)
                    interval = 2;
                else
                    interval = 1;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        count++;
                        regLine = (int)(LeftLine_2.Line_k * LeftMargin_2[i].row + LeftLine_2.Line_b);
                        if (regLine > 185)
                            regLine = 185;
                        else if (regLine < 0)
                            regLine = 0;
                        sum += my_fabs(LeftMargin_2[i].line - regLine);
                        //SetText_1("regLine = " + regLine + " LeftMargin_2[" + i + "].line = " + LeftMargin_2[i].line);
                        //SetText_1("sum = " + sum);
                        Mask2[LeftMargin_2[i].row, regLine] = 1;
                    }
                }
                sum /= count;
                //SetText_1("LeftLine 2 Str Error = " + sum);
                //LeftLine_2.RecStrError = sum;
            }
            else 
            {
                //SetText_1("End Point RightMargin_2[" + endPoint + "] : Row = " + RightMargin_2[endPoint].row + " Line = " + RightMargin_2[endPoint].line);
                if (endPoint - startPoint >= 40)
                    interval = 4;
                else if (endPoint - startPoint >= 20)
                    interval = 2;
                else
                    interval = 1;
                for (i = startPoint; i <= endPoint; i++)
                {
                    if (i % interval == 0)
                    {
                        count++;
                        regLine = (int)(RightLine_2.Line_k * RightMargin_2[i].row + RightLine_2.Line_b);
                        if (regLine > 185)
                            regLine = 185;
                        else if (regLine < 0)
                            regLine = 0;
                        sum += my_fabs(RightMargin_2[i].line - regLine);
                        //SetText_1("regLine = " + regLine + " RightMargin_2["+i+"].row = "+ RightMargin_2[i].row+" RightMargin_2[" + i + "].line = " + RightMargin_2[i].line);
                        //SetText_1("sum = " + sum);
                        Mask2[RightMargin_2[i].row, regLine] = 1;
                    }
                }
                sum /= count;
                //SetText_1("RightLine 2 Str Error = " + sum);
                //RightLine_2.RecStrError = sum;
            }
            return sum;
        }
        float MarginStrErrorCale_1(int startPoint, int endPoint, Scan_LineType Line_RorL)//0为左，1为右
        {
            if (startPoint < 0)
                startPoint = 0;
            if (endPoint < 0)
                endPoint = 0;
            if (startPoint == endPoint)
                return 0;
            //SetText_1("MarginStrError 1 Cale Type = "+ Line_RorL+ " startPoint = " + startPoint+ " endPoint = " + endPoint);
            MarginReg_1((byte)startPoint, (byte)endPoint, Line_RorL);
            ///SetText_1("Error "+MarginStrCmp_1((byte)startPoint, (byte)endPoint, Line_RorL));
            return MarginStrCmp_1((byte)startPoint, (byte)endPoint, Line_RorL);
        }
        float MarginStrErrorCale_2(int startPoint, int endPoint, Scan_LineType Line_RorL)//0为左，1为右
        {
            if (startPoint < 0)
                startPoint = 0;
            if (endPoint < 0)
                endPoint = 0;
            if (startPoint == endPoint)
                return 0;
            //SetText_1("MarginStrError 2 Cale Type = " + Line_RorL+ " startPoint = " + startPoint+ " endPoint = " + endPoint);
            MarginReg_2((byte)startPoint, (byte)endPoint, Line_RorL);
            //SetText_1("Error " + MarginStrCmp_2((byte)startPoint, (byte)endPoint, Line_RorL));
            return MarginStrCmp_2((byte)startPoint, (byte)endPoint, Line_RorL);
        }
        #endregion
        #region 赛道扫线判断函数及变量
        public enum RoadTypeEnum { Common, Straight, Slow_Turn_L,Slow_Turn_R,Turn_L, Turn_R, CrossLine, Circle_L, Circle_R, Slope,Small_Slope, Garage_L, Garage_R, Out_Garage_L, Out_Garage_R,Meeting }; // 道路类型
        public static RoadTypeEnum RoadType = RoadTypeEnum.Common;

        public enum CommonTypeEnum { Short_Straight,Small_Turn }; // 普通赛道：短直道、小弯
        public static CommonTypeEnum CommonType = CommonTypeEnum.Short_Straight;

        public enum TurnTypeEnum { Mid_Turn,Big_Turn }; // 弯道：中弯、大弯
        public static TurnTypeEnum TurnType = TurnTypeEnum.Mid_Turn;

        public enum CrossLineTypeEnum { Short_Cross, Long_Cross }; // 十字：短十字、长十字
        public static CrossLineTypeEnum CrossLineType = CrossLineTypeEnum.Short_Cross;
        
        public enum Circle { Circle_0, Circle_1, Circle_2, Circle_3, Circle_4, Circle_5, Circle_6, Circle_7, Circle_8 }; // 环岛状态
        public static Circle Circle_State = Circle.Circle_0;
        public enum Circle_RoadEnum { Circle_Common,Circle_Straight,Circle_Turn }; // 环岛的路段类型
        public static Circle_RoadEnum Circle_Road = Circle_RoadEnum.Circle_Common;

        public enum Garage { Garage_0, Garage_1, Garage_2, Garage_3, Garage_4, Garage_5, Garage_6, Garage_7, Garage_8 }; // 入车库状态
        public static Garage Garage_State = Garage.Garage_0;

        public enum Out_Garage { Out_Garage_0, Out_Garage_1, Out_Garage_2, Out_Garage_3, Out_Garage_4 }; // 出车库状态
        public static Out_Garage Out_Garage_State = Out_Garage.Out_Garage_0;

        public enum Meeting_st { Meeting_0, Meeting_1, Meeting_2, Meeting_3, Meeting_4 }; // 会车状态
        public static Meeting_st Meeting_State = Meeting_st.Meeting_0;

        public enum Slope_st { Slope_0, Slope_1, Slope_2, Slope_3, Slope_4 }; // 坡道状态
        public static Slope_st Slope_State = Slope_st.Slope_0;
        float L_forwad_line_k, L_front_line_k, L_forward_line_error, L_front_line_error;
        float R_forwad_line_k, R_front_line_k, R_forward_line_error, R_front_line_error;
        int AllSpecRoadCnt = -1;
        void CrossLine_Scan() // 十字赛道的扫描处理
        {
            SetText_1("In CrossLine");
            SetText_1("****************************CrossLine Scan Start*************************");
            if (LeftMargin_1[0].row != 0 && RightMargin_1[0].row != 0)
            {
                Traverse_Agl_1(Scan_LineType.LeftType, 65);
                Traverse_Agl_1(Scan_LineType.RightType, 65);
                NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftLine_1.Agl_Line + 1) , (byte)(LeftLine_1.Agl_Row + 10), 69);
                Scan_And_Traverse_Agl_2(Scan_LineType.LeftType, 0, 100);
                NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical,(byte)(RightLine_1.Agl_Line - 1) , (byte)(RightLine_1.Agl_Row + 10), 69);
                Scan_And_Traverse_Agl_2(Scan_LineType.RightType, 0, 100);
            }
            else if (LeftMargin_1[0].row != 0 && RightMargin_1[0].row == 0)
            {
                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                Traverse_Agl_1(Scan_LineType.LeftType, 65);

                NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical,(byte)(LeftLine_1.Agl_Line - 10) , (byte)(LeftLine_1.Agl_Row + 5), 69);
                Scan_And_Traverse_Agl_2(Scan_LineType.LeftType, 0, 100);
                if (RightMargin_2[0].row == 0)
                {
                    NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 1, (byte)(LeftLine_2.Agl_Row!=0?LeftLine_1.Agl_Row +5:10), 69);
                    EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                    if(RightLine_2.PointCnt<20&&RightLine_2.EndLine<LeftLine_1.Agl_Row)
                    {
                        NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 1, (byte)(RightMargin_2[RightLine_2.PointCnt].row+5), 69);
                        EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                    }
                    Traverse_Agl_2(Scan_LineType.RightType, 65);
                }
            }
            else if (LeftMargin_1[0].row == 0 && RightMargin_1[0].row != 0)
            {
                EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                Traverse_Agl_1(Scan_LineType.RightType, 65);

                NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical,  (byte)(RightLine_1.Agl_Line + 10) , (byte)(RightLine_1.Agl_Row + 5), 69);
                Scan_And_Traverse_Agl_2(Scan_LineType.RightType, 0, 100);
                if (LeftMargin_2[0].row==0)
                {
                    NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, (byte)(RightLine_2.Agl_Row!=0?RightLine_1.Agl_Row +5:10), 69);
                    EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                    if (LeftLine_2.PointCnt < 20 && LeftLine_2.EndLine < RightLine_1.Agl_Row)
                    {
                        NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, (byte)(LeftMargin_2[LeftLine_2.PointCnt].row + 5), 69);
                        EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                    }
                    Traverse_Agl_2(Scan_LineType.LeftType, 65);
                }
            }
            else if (LeftMargin_1[0].row == 0 && RightMargin_1[0].row == 0)
            {
                NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 165, 1, 69);
                EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 20, 1, 69);
                EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                Traverse_Agl_2(Scan_LineType.LeftType, 65);
                //LeftLine_2.AglStrError = MarginStrErrorCale_2(0, LeftLine_2.Agl_PointNum, Scan_LineType.LeftType);
                //LeftLine_2.RecStrError = MarginStrErrorCale_2(0, LeftLine_2.Rec_Point, Scan_LineType.LeftType);
                Traverse_Agl_2(Scan_LineType.RightType, 65);
                //RightLine_2.AglStrError = MarginStrErrorCale_2(0, RightLine_2.Agl_PointNum, Scan_LineType.RightType);
                //RightLine_2.RecStrError = MarginStrErrorCale_2(0, RightLine_2.Rec_Point, Scan_LineType.RightType);
            }
            /*if (RightLine_2.Turn_Cnt + LeftLine_2.Turn_Cnt <= 20)
                CrossLineType = CrossLineTypeEnum.Long_Cross;
            else
                CrossLineType = CrossLineTypeEnum.Short_Cross;*/
            SetText_1("****************************CrossLine Scan End*************************");
        }
        void Circle_Scan() // 环岛赛道的扫描处理
        {
            SetText_1("****************************Circle_Scan Start*************************");
            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
            if (RoadType == RoadTypeEnum.Circle_L)
            {
                //if (Error_Flag == 0)
                //{
                if (Circle_State == Circle.Circle_0 && LeftMargin_1[0].row == 0)
                {
                    Circle_State = Circle.Circle_1;
                }
                else if (Circle_State == Circle.Circle_1 && LeftMargin_1[0].row != 0 && LeftLine_1.PointCnt > 60)
                {
                    Circle_State = Circle.Circle_2;
                }
                else if (Circle_State == Circle.Circle_4
                        && RightMargin_1[0].row == 0
                        && NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 20) == 0)
                {
                    Circle_State = Circle.Circle_5;
                }
                else if (Circle_State == Circle.Circle_5
                && J_Pixels[20][1] == 0 && J_Pixels[21][1] == 0 && J_Pixels[22][1] == 0 && J_Pixels[23][1] == 0 && J_Pixels[24][1] == 0 && J_Pixels[25][1] == 0)
                {
                    Circle_State = Circle.Circle_6;
                }
                else if (Circle_State == Circle.Circle_7 && LeftMargin_1[0].row != 0)
                {
                    Circle_State = Circle.Circle_0;
                    RoadType = RoadTypeEnum.Common;
                    return;
                }
                //}
                switch (Circle_State)
                {
                    case Circle.Circle_0:
                        Traverse_Agl_1(Scan_LineType.LeftType, 65);
                        if (Garage_Judge(0) >= 1)
                        {
                            RoadType = RoadTypeEnum.Garage_L;
                            Garage_Scan();
                            return;
                        }
                        if (RightMargin_1[0].row == 0 && NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50) == 1)
                        {
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                        }
                        NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, LeftLine_1.Agl_Line, (byte)(LeftLine_1.Agl_Row + 10), 69);
                        Scan_And_Traverse_Circle_Agl_2(Scan_LineType.LeftType, 0, 100);
                        break;
                    case Circle.Circle_1:
                        if (RightMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 40);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                        }
                        NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 160, 1, 69);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                        Traverse_Agl_1(Scan_LineType.LeftType, 69);
                        break;
                    case Circle.Circle_2:
                        if (RightMargin_1[0].row == 0 && NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 0, 1, 30) == 1)
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);

                        Traverse_Circle_Agl_1(Scan_LineType.LeftType, 65);
                        NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftLine_1.Agl_Line -5), (byte)(LeftLine_1.Agl_Row + 5), 69);
                        Scan_And_Traverse_Agl_2(Scan_LineType.LeftType, 0, 120);

                        if ((LeftLine_1.Agl_Row != 0 && (LeftLine_1.Agl_Row < 15 || LeftLine_1.Agl_Line > 160))
                            || LeftMargin_1[0].row == 0)
                        {
                            Circle_State = Circle.Circle_3;
                            if (LeftLine_1.Agl_Row > 20)
                            {
                                NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftLine_1.Agl_Line - 10), (byte)(LeftLine_1.Agl_Row + 5), 69);
                            }
                            else
                            {
                                NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 135, (byte)(LeftLine_1.Agl_Row + 5), 69);
                            }
                            Scan_And_Traverse_Agl_2(Scan_LineType.LeftType, 0, 100);
                            LeftLine_2.PointCnt = 0;

                            if (LeftLine_2.Agl_Row != 0)
                            {
                                NewSeedSet_2(Scan_LineType.RightType, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                                EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                            }
                        }
                        break;
                    case Circle.Circle_3:
                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 20);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        }
                        Traverse_Circle_Agl_1(Scan_LineType.LeftType, 65);
                        if (LeftLine_1.Agl_Row > 20)
                        {
                            NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftLine_1.Agl_Line - 20), (byte)(LeftLine_1.Agl_Row + 5), 69);
                        }
                        else
                        {
                            NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 135, (byte)(LeftLine_1.Agl_Row + 5), 69);
                        }
                        Scan_And_Traverse_Loose_Agl_2(Scan_LineType.LeftType, 0, 148);

                        if (LeftLine_2.Agl_Row != 0)
                        {
                            NewSeedSet_2(Scan_LineType.RightType, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                            EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                        }
                        else
                        {
                            NewSeedSet_2(Scan_LineType.RightType, LeftMargin_2[LeftLine_2.PointCnt].row, LeftMargin_2[LeftLine_2.PointCnt].line);
                            EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                        }
                        LeftLine_2.PointCnt = 0;

                        if (RightMargin_1[0].row != 0)
                        {
                            MarginReg_1(1, RightLine_1.Rec_Point, Scan_LineType.RightType);
                            MarginStrCmp_1(0, RightLine_1.Rec_Point, Scan_LineType.RightType);
                            setText用户自定义("RightLine_1.line k = " + RightLine_1.Line_k);
                            if (RightLine_1.Line_k > 1.7)
                            {
                                Circle_State = Circle.Circle_4;
                            }
                        }
                        break;
                    case Circle.Circle_4:
                        if(RightMargin_1[0].row==0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 20);
                        }
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                        if (((Traverse_Agl_1(Scan_LineType.RightType, 65) == 1 && RightLine_1.BroadWire_Cnt > 20)) || RightLine_1.PointCnt <= 60)
                        {
                            if (NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 145, (byte)(RightLine_1.Agl_Row + 3), 69) == 1)
                            {
                                EightRegionScanLine_2(Scan_LineType.RightType, 0, 60);
                                if (RightLine_2.Error == 1)
                                {
                                    NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 5, (byte)(RightLine_1.Agl_Row + 3), 69);
                                    EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                                }
                            }
                        }
                        break;
                    case Circle.Circle_5:
                        if (RightMargin_1[0].row == 0)
                        {
                            if (NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 105, 1, 69) == 0)
                                NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 145, 1, 69);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 60);
                            if (RightLine_1.Error == 1)
                            {
                                NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 5, 1, 60);
                                EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                            }
                        }
                        break;
                    case Circle.Circle_6:
                        if (RightMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 15, 1, 69);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        }
                        MarginStrErrorCale_1(0, RightLine_1.Rec_Point, Scan_LineType.RightType);
                        Traverse_Circle_Agl_1(Scan_LineType.RightType, 60);
                        SetText_2("MarRightLine_1.Line_k = " + RightLine_1.Line_k);
                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, 1, 93, 184);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        }
                        //SetText_1("Traverse_Circle_Agl_1(Scan_LineType.LeftType, 69) = "+ Traverse_Circle_Agl_1(Scan_LineType.LeftType, 69));
                        Traverse_Circle_Agl_1(Scan_LineType.LeftType, 69);
                        if (RightLine_1.Line_k < 3)
                        {
                            if (RightLine_1.Line_k > 2)
                            {
                                if (LeftMargin_1[LeftLine_1.PointCnt].line == 185)
                                {
                                    NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftMargin_1[LeftLine_1.PointCnt].line < 180 ? LeftMargin_1[LeftLine_1.PointCnt].line : 180), (byte)(LeftMargin_1[LeftLine_1.PointCnt].row + 3), 69);
                                }
                                else
                                {
                                    NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftLine_1.Agl_Line + 20 < 180 ? LeftLine_1.Agl_Line + 20 : 180), (byte)(LeftLine_1.Agl_Row + 20), 69);
                                }
                            }
                            else
                            {
                                NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(LeftLine_1.Agl_Line - 5 < 180 ? LeftLine_1.Agl_Line - 5 : 180), (byte)(LeftLine_1.Agl_Row + 10), 69);
                            }
                            EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                        }
                        if (Traverse_Agl_2(Scan_LineType.LeftType, 69) == 1)
                        {
                            if (LeftLine_2.Agl_Row < 30)
                                Circle_State = Circle.Circle_7;
                        }
                        else if (Traverse_Circle_Agl_2(Scan_LineType.LeftType, 69) == 1)
                        {

                        }
                        else
                            Traverse_Loose_Agl_2(Scan_LineType.LeftType, 69);
                        break;
                    case Circle.Circle_7:
                        if (NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 170, 1, 65) == 1)
                            Scan_And_Traverse_Agl_2(Scan_LineType.LeftType, 0, 100);
                        break;
                    default: break;
                }
            }
            else
            {
                //if (Error_Flag == 0)
                //{
                if (Circle_State == Circle.Circle_0 && RightMargin_1[0].row == 0)
                {
                    Circle_State = Circle.Circle_1;
                }
                else if (Circle_State == Circle.Circle_1 && RightMargin_1[0].row != 0 && RightLine_1.PointCnt > 60)
                {
                    Circle_State = Circle.Circle_2;
                }
                else if (Circle_State == Circle.Circle_4
                    && LeftMargin_1[0].row == 0
                    && NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 20) == 0)
                {
                    Circle_State = Circle.Circle_5;
                }
                else if (Circle_State == Circle.Circle_5
                && J_Pixels[20][184] == 0 && J_Pixels[21][184] == 0 && J_Pixels[22][184] == 0 && J_Pixels[23][184] == 0 && J_Pixels[24][184] == 0 && J_Pixels[25][184] == 0)
                {
                    Circle_State = Circle.Circle_6;
                }
                else if (Circle_State == Circle.Circle_7 && RightMargin_1[0].row != 0)
                {
                    Circle_State = Circle.Circle_0;
                    RoadType = RoadTypeEnum.Common;
                    return;
                }
                switch (Circle_State)
                {
                    case Circle.Circle_0:
                        Traverse_Agl_1(Scan_LineType.RightType, 65);
                        if (Garage_Judge(1) >= 1)
                        {
                            RoadType = RoadTypeEnum.Garage_R;
                            Garage_Scan();
                            return;
                        }
                        if (LeftMargin_1[0].row == 0 && NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50) == 1)
                        {
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                        }
                        NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, RightLine_1.Agl_Line, (byte)(RightLine_1.Agl_Row + 1), 69);
                        Scan_And_Traverse_Circle_Agl_2(Scan_LineType.RightType, 0, 100);
                        break;
                    case Circle.Circle_1:
                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 40);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                        }
                        NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 25, 1, 69);
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                        Traverse_Agl_1(Scan_LineType.RightType, 69);
                        break;
                    case Circle.Circle_2:
                        if (LeftMargin_1[0].row == 0 && NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 30) == 1)
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);

                        Traverse_Circle_Agl_1(Scan_LineType.RightType, 65);
                        NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(RightLine_1.Agl_Line + 5), (byte)(RightLine_1.Agl_Row + 5), 69);
                        Scan_And_Traverse_Agl_2(Scan_LineType.RightType, 0, 120);

                        if ((RightLine_1.Agl_Row != 0 && (RightLine_1.Agl_Row < 15 || RightLine_1.Agl_Line < 25))
                            || RightMargin_1[0].row == 0)
                        {
                            Circle_State = Circle.Circle_3;
                            if (RightLine_1.Agl_Row > 20)
                            {
                                NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(RightLine_1.Agl_Line + 20), (byte)(RightLine_1.Agl_Row + 5), 69);
                            }
                            else
                            {
                                NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 50, (byte)(RightLine_1.Agl_Row + 5), 69);
                            }
                            Scan_And_Traverse_Agl_2(Scan_LineType.RightType, 0, 100);
                            RightLine_2.PointCnt = 0;

                            if (RightLine_2.Agl_Row != 0)
                            {
                                NewSeedSet_2(Scan_LineType.LeftType, RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                                EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                            }
                        }
                        break;
                    case Circle.Circle_3:
                        if (RightMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 20);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        }
                        Traverse_Circle_Agl_1(Scan_LineType.RightType, 65);
                        if (RightLine_1.Agl_Row > 20)
                        {
                            NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(RightLine_1.Agl_Line + 10), (byte)(RightLine_1.Agl_Row + 5), 69);
                        }
                        else
                        {
                            NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 50, (byte)(RightLine_1.Agl_Row + 5), 69);
                        }
                        Scan_And_Traverse_Loose_Agl_2(Scan_LineType.RightType, 0, 148);

                        if (RightLine_2.Agl_Row != 0)
                        {
                            NewSeedSet_2(Scan_LineType.LeftType, RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                            EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                        }
                        else
                        {
                            NewSeedSet_2(Scan_LineType.LeftType, RightMargin_2[RightLine_2.PointCnt].row, RightMargin_2[RightLine_2.PointCnt].line);
                            EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                        }
                        RightLine_2.PointCnt = 0;

                        if (LeftMargin_1[0].row != 0)
                        {
                            MarginReg_1(0, LeftLine_1.Rec_Point, Scan_LineType.LeftType);
                            setText用户自定义("LeftLine_1.line k = " + LeftLine_1.Line_k);
                            if (LeftLine_1.Line_k < -1.7)
                            {
                                Circle_State = Circle.Circle_4;
                            }
                        }
                        break;
                    case Circle.Circle_4:
                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 20);
                        }
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                        if (((Traverse_Agl_1(Scan_LineType.LeftType, 65) == 1 && LeftLine_1.BroadWire_Cnt > 20)) || LeftLine_1.PointCnt <= 60)
                        {
                            if (NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 40, (byte)(LeftLine_1.Agl_Row + 3), 69) == 1)
                            {
                                EightRegionScanLine_2(Scan_LineType.LeftType, 0, 60);
                                if (LeftLine_2.Error == 1)
                                {
                                    NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 180, (byte)(LeftLine_1.Agl_Row + 3), 69);
                                    EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                                }
                            }
                        }
                        break;
                    case Circle.Circle_5:
                        if (LeftMargin_1[0].row == 0)
                        {
                            if (NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 80, 1, 69) == 0)
                                NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 40, 1, 69);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 60);
                            if(LeftLine_1.Error==1)
                            {
                                NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 180, 1, 60);
                                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                            }
                        }
                        break;
                    case Circle.Circle_6:
                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 180, 1, 69);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        }
                        MarginStrErrorCale_1(0, LeftLine_1.Rec_Point, Scan_LineType.LeftType);
                        Traverse_Circle_Agl_1(Scan_LineType.LeftType, 60);
                        SetText_2("MarLeftLine_1.Line_k = " + LeftLine_1.Line_k);
                        if (RightMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, 1, 1, 93);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        }
                        Traverse_Circle_Agl_1(Scan_LineType.RightType, 69);
                        if (LeftLine_1.Line_k > -3)
                        {
                            if (LeftLine_1.Line_k < -2)
                            {
                                if (RightMargin_1[RightLine_1.PointCnt].line == 0)
                                {
                                    NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(RightMargin_1[RightLine_1.PointCnt].line > 5 ? RightMargin_1[RightLine_1.PointCnt].line : 5), (byte)(RightMargin_1[RightLine_1.PointCnt].row + 3), 69);
                                }
                                else
                                {
                                    NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(RightLine_1.Agl_Line - 20 > 5 ? RightLine_1.Agl_Line - 20 : 5), (byte)(RightLine_1.Agl_Row + 20), 69);
                                }
                            }
                            else
                            {
                                NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(RightLine_1.Agl_Line + 5 > 5 ? RightLine_1.Agl_Line + 5 : 5), (byte)(RightLine_1.Agl_Row + 10), 69);
                            }
                            //SetText_1("RightLine_1.Agl_Line = " + RightLine_1.Agl_Line + " LeftMargin_1[0].line = " + LeftMargin_1[0].line);
                            EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                        }
                        if (Traverse_Agl_2(Scan_LineType.RightType, 69) == 1)
                        {
                            if (RightLine_2.Agl_Row < 30)
                                Circle_State = Circle.Circle_7;
                        }
                        else if (Traverse_Circle_Agl_2(Scan_LineType.RightType, 69) == 1)
                        {

                        }
                        else
                            Traverse_Loose_Agl_2(Scan_LineType.RightType, 69);
                        break;
                    case Circle.Circle_7:
                        if (NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 15, 1, 65) == 1)
                            Scan_And_Traverse_Agl_2(Scan_LineType.RightType, 0, 100);
                        break;
                    default: break;
                }
            }
            SetText_1("Circle_State = " + Circle_State);
            setText用户自定义("Circle_State = " + Circle_State);
            SetText_1("****************************Circle_Scan End*************************");
            SetText_1(" ");
        }
        void Turn_Scan() // 弯道的扫描处理
        {
            SetText_1(" ");
            SetText_1("****************************Turn Scan Start*************************");
            if(LeftMargin_1[0].row!=0)
            {
                EightRegionScanLine_1(Scan_LineType.LeftType, (byte)(LeftLine_1.PointCnt-1), 179);
                //Traverse_Circle_Agl_1(Scan_LineType.LeftType, 69);
                Traverse_Turn_Agl_1(Scan_LineType.LeftType, 69);
            }
            if(RightMargin_1[0].row!=0)
            {
                EightRegionScanLine_1(Scan_LineType.RightType, (byte)(RightLine_1.PointCnt-1), 179);
                Traverse_Turn_Agl_1(Scan_LineType.RightType, 69);
            }
            if (LeftMargin_1[0].row == 0)
            {
                if (RightLine_1.Agl_Row != 0)
                {
                    NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, (byte)((RightLine_1.EndLine - 1) > 15 ? (RightLine_1.EndLine - 1) : 15));
                }
                else
                {
                    NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, (byte)((RightLine_1.EndLine - 10) > 15 ? (RightLine_1.EndLine - 10) : 15));
                }
                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 179);
                Traverse_Turn_Agl_1(Scan_LineType.LeftType, 69);
            }
            if (RightMargin_1[0].row == 0)
            {
                if (LeftLine_1.Agl_Row != 0)
                {
                    NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, (byte)((LeftLine_1.EndLine - 1) > 15 ? (LeftLine_1.EndLine - 1) : 15));
                }
                else
                {
                    NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, (byte)((LeftLine_1.EndLine - 10) > 15 ? (LeftLine_1.EndLine - 10) : 15));
                }
                EightRegionScanLine_1(Scan_LineType.RightType, 0, 179);
                Traverse_Turn_Agl_1(Scan_LineType.RightType, 69);
            }
            setText用户自定义("RecStrError Sum = " + (LeftLine_1.RecStrError+ RightLine_1.RecStrError));
            if (((LeftLine_1.Agl_Row != 0 && LeftLine_1.Agl_2_Row != 0) || (RightLine_1.Agl_Row != 0 && RightLine_1.Agl_2_Row != 0))
                && LeftLine_1.Agl_Row != 0 && RightLine_1.Agl_Row != 0)
            {
                TurnType = TurnTypeEnum.Mid_Turn;
            }
            else
            {
                TurnType = TurnTypeEnum.Big_Turn;
            }
            setText用户自定义("TurnType = "+ TurnType);
            SetText_1("****************************Turn Scan End*************************");
            SetText_1(" ");
        }
        byte Garage_Judge(byte i) // 车库的扫描处理
        {
            int y = 0, x = 0;
            byte count = 0, pixel_cnt = 0, zebraWidth = 0, m = 0, Limit = 0; ;
            byte last_x=0;
            //左边车库
            if (i == 0)
            {
                SetText_1("//Left Garage Judge Start");
                for (OlRow = LeftLine_1.Agl_Row; OlRow <= (LeftLine_1.Agl_Row > 10 ? LeftLine_1.Agl_Row + 20 : LeftLine_1.Agl_Row + 30) && OlRow <= 69; OlRow++)
                {
                    for (OlLine = (byte)(LeftLine_1.Agl_Line - 1); OlLine >= 75; OlLine--)
                    {
                        //SetText_1("Zerba Scan :"+ OlRow+" "+ OlLine);
                        if (J_Pixels[OlRow][OlLine] == white && J_Pixels[OlRow][OlLine - 1] == black)
                        {
                            //SetText_1("Find Zebra Point: " + OlRow + " " + OlLine);
                            y = (byte)(OlRow);
                            x = (byte)(OlLine - 1);
                            break;
                        }
                    }
                    if (y != 0)
                        break;
                }
                if (y == 0)
                {
                    SetText_1("Lost Zebra Point");
                    return 0;
                }
                Limit = 0;
                for (m = 0; m < 5; m++)
                {
                    //SetText_1("m = " + m);
                    count = pixel_cnt = 0;
                    for (x = LeftLine_1.Agl_Line-1; x > Limit; x--)
                    {
                        if (J_Pixels[y + m][x + 1] == white && J_Pixels[y + m][x] == black)
                        {
                            //SetText_1("Zebra Scan Start: " + (y + m) + " " + x);
                            Mask2[y + m, x] = 1;
                            pixel_cnt = 1;
                        }
                        else if (J_Pixels[y + m][x + 1] == black && J_Pixels[y + m][x] == black)
                        {
                            //SetText_1("Zebra Add: " + (y + m) + " " + x);
                            Mask2[y + m, x] = 1;
                            pixel_cnt++;
                        }
                        else if (J_Pixels[y + m][x + 1] == black && J_Pixels[y + m][x] == white)
                        {
                            if (count == 0 && pixel_cnt >= 1 && pixel_cnt <= 10)
                            {
                                zebraWidth = pixel_cnt;
                                //SetText_1("zebraWidth = " + zebraWidth);
                                //SetText_1("zebraStart Point: " + (y + m) + " " + x);
                                count = 1;
                                GarageLine = (byte)x;
                                last_x = (byte)x;
                            }
                            else if (count > 0 && my_fabs(pixel_cnt - zebraWidth) <= 4)
                            {
                                //SetText_1("Zebra End: " + (y + m) + " " + (x - 1));
                                if (my_fabs(last_x - x )> 50)
                                    break;
                                else
                                    last_x = (byte)x;
                                count++;
                            }
                            else
                            {
                                //SetText_1("Zebra Error: " + (y + m) + " " + (x - 1));
                            }
                        }
                        if (pixel_cnt >= 15)
                        {
                            Limit = (byte)(x + pixel_cnt);
                            //SetText_1("Limit = "+ Limit);
                            break;
                        }
                    }
                    //SetText_1("Zebra Count = " + count);
                    if (count >= 5)
                    {
                        SetText_1("Proved to be Garage , count "+ count + " , Garage Line " + (y + m));
                        GarageRow = (byte)(y + m);
                        return (byte)(y + m);
                    }
                }
            }
            else  //右边车库
            {
                SetText_1("//Right Garage Judge Start");
                for (OlRow = RightLine_1.Agl_Row; OlRow <= (RightLine_1.Agl_Row > 10 ? RightLine_1.Agl_Row + 20 : RightLine_1.Agl_Row + 30) && OlRow <= 69; OlRow++)
                {
                    for (OlLine = (byte)(RightLine_1.Agl_Line + 1); OlLine <= 110; OlLine++)
                    {
                        //SetText_1("Zerba Scan :"+ OlRow+" "+ OlLine);
                        if (J_Pixels[OlRow][OlLine] == black && J_Pixels[OlRow][OlLine - 1] == white)
                        {
                            //x = (byte)(OlLine-1);
                            //SetText_1("Find Zebra Point: " + OlRow + " " + OlLine);
                            y = (byte)(OlRow);
                            break;
                        }
                    }
                    if (y != 0)
                        break;
                }
                if (y == 0)
                {
                    SetText_1("Lost Zebra Point");
                    return 0;
                }
                Limit = 185;
                for (m = 0; m < 5; m++)
                {
                    //SetText_1("m = " + m);
                    count = pixel_cnt = 0;
                    for (x = RightLine_1.Agl_Line+1; x < Limit; x++)
                    {
                        if (J_Pixels[y + m][x - 1] == white && J_Pixels[y + m][x] == black)
                        {
                            //SetText_1("Zebra Scan Start: " + y + " " + x);
                            pixel_cnt = 1;
                            Mask1[y + m, x] = 1;
                        }
                        else if (J_Pixels[y + m][x - 1] == black && J_Pixels[y + m][x] == black)
                        {
                            //SetText_1("Zebra Add: " + y + " " + x);
                            pixel_cnt++;
                            Mask1[y + m, x] = 1;
                        }
                        else if (J_Pixels[y + m][x - 1] == black && J_Pixels[y + m][x] == white)
                        {
                            if (count == 0 && pixel_cnt >= 1 && pixel_cnt <= 10)
                            {
                                zebraWidth = pixel_cnt;
                                //SetText_1("zebraWidth = " + zebraWidth);
                                //SetText_1("zebraStart Point: " + (y + m) + " " + x);
                                count = 1;
                                GarageLine = (byte)x;
                                last_x = (byte)x;
                            }
                            else if (count > 0 && my_fabs(pixel_cnt - zebraWidth) <= 4)
                            {
                                if (my_fabs(x - last_x) > 50)
                                    break;
                                else
                                    last_x = (byte)x;
                                //SetText_1("Zebra End: " + y + " " + (x - 1));
                                count++;
                                
                            }
                            else
                            {
                                //SetText_1("Zebra Error: " + y + " " + (x - 1));
                            }
                        }
                        if (pixel_cnt >= 15)
                        {
                            //SetText_1("y = "+(y + m)+" x = "+x);
                            Limit = (byte)(x - pixel_cnt);
                            //SetText_1("Limit = " + Limit);
                            break;
                        }
                    }
                    //SetText_1("Zebra Count = " + count);
                    if (count >= 5)
                    {
                        SetText_1("Proved to be Garage , count " + count + " , Garage Line " + (y + m));
                        GarageRow = (byte)(y + m);
                        return (byte)(y + m);
                    }
                }
            }
            //setText用户自定义("Garage Judge!!!!!!!");
            SetText_1("Not Proved to be Garage");
            SetText_1("//Garage Judge End");
            GarageRow = GarageLine = 0;
            return 0;
        }
        void Meeting_Scan() // 会车的扫描处理
        {
            SetText_1("****************************Meeting_Scan Start*************************");
            if (Meeting_State == Meeting_st.Meeting_0)
            {

            }
            else if (Meeting_State == Meeting_st.Meeting_1)
            {

            }
            switch (Meeting_State)
            {
                case Meeting_st.Meeting_0:
                    Meeting_BiasSeed_Find(2, 60, 2, 0);
                    Car_RegionScanLine(Scan_LineType.LeftType, 0, 50);
                    Car_RegionScanLine(Scan_LineType.RightType, 0, 50);
                    Traverse_CarAgl(Scan_LineType.LeftType, 69);
                    Traverse_CarAgl(Scan_LineType.RightType, 69);
                    if (Car_widthCale(MeetingMode) == 1)
                    {
                        if (Car_RightLine.Agl_Row != 0 && Car_LeftLine.Agl_Row != 0)
                        {
                            CarTrailRow = (byte)((Car_RightLine.Agl_Row + Car_LeftLine.Agl_Row) / 2);
                            CarTrailLine = (byte)((Car_RightLine.Agl_Line + Car_LeftLine.Agl_Line) / 2);
                        }
                        else if (Car_LeftLine.Agl_Row != 0
                            && Car_RightMargin[Car_RightLine.PointCnt].row != 0)
                        {
                            CarTrailRow = (byte)((Car_RightMargin[Car_RightLine.PointCnt].row + Car_LeftLine.Agl_Row) / 2);
                            CarTrailLine = (byte)((Car_RightMargin[Car_RightLine.PointCnt].line + Car_LeftLine.Agl_Line) / 2);
                        }
                        else if (Car_RightLine.Agl_Row != 0
                            && Car_LeftMargin[Car_LeftLine.PointCnt].row != 0)
                        {
                            CarTrailRow = (byte)((Car_LeftMargin[Car_LeftLine.PointCnt].row + Car_RightLine.Agl_Row) / 2);
                            CarTrailLine = (byte)((Car_LeftMargin[Car_LeftLine.PointCnt].line + Car_RightLine.Agl_Line) / 2);
                        }
                    }
                    else
                    {
                        RoadType = RoadTypeEnum.Common;
                        if (Meeting_Judge() == 0)
                        {
                            RoadType = RoadTypeEnum.Common;
                            Meeting_State = Meeting_st.Meeting_0;
                            Initial_Seed_Find(Scan_LineType.LeftType, 10);
                            Initial_Seed_Find(Scan_LineType.RightType, 10);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                            return;
                        }
                    }
                    break;
                case Meeting_st.Meeting_1:
                    Meeting_BiasSeed_Find(2, 60, 2, 0);
                    Car_RegionScanLine(Scan_LineType.LeftType, 0, 50);
                    Car_RegionScanLine(Scan_LineType.RightType, 0, 50);
                    Traverse_CarAgl(Scan_LineType.LeftType, 69);
                    Traverse_CarAgl(Scan_LineType.RightType, 69);
                    if (Car_RightLine.Agl_Row != 0 && Car_LeftLine.Agl_Row != 0)
                    {
                        CarTrailRow = (byte)((Car_RightLine.Agl_Row + Car_LeftLine.Agl_Row) / 2);
                        CarTrailLine = (byte)((Car_RightLine.Agl_Line + Car_LeftLine.Agl_Line) / 2);
                    }
                    else if (Car_LeftLine.Agl_Row != 0)
                    {
                        CarTrailRow = (byte)((Car_RightMargin[Car_RightLine.PointCnt].row + Car_LeftLine.Agl_Row) / 2);
                        CarTrailLine = (byte)((Car_RightMargin[Car_RightLine.PointCnt].line + Car_LeftLine.Agl_Line) / 2);
                    }
                    else if (Car_RightLine.Agl_Row != 0)
                    {
                        CarTrailRow = (byte)((Car_LeftMargin[Car_LeftLine.PointCnt].row + Car_RightLine.Agl_Row) / 2);
                        CarTrailLine = (byte)((Car_LeftMargin[Car_LeftLine.PointCnt].line + Car_RightLine.Agl_Line) / 2);
                    }
                    else
                    {
                        CarTrailRow = 1;
                        CarTrailLine = 93;
                    }
                    SetText_1("Trail Row = " + CarTrailRow + " Trail Line " + CarTrailLine);
                    break;
                case Meeting_st.Meeting_2:
                    Car_Seed_Find(93, 2, 20);
                    Car_RegionScanLine(Scan_LineType.LeftType, 0, 50);
                    Car_RegionScanLine(Scan_LineType.RightType, 0, 50);
                    Traverse_CarAgl(Scan_LineType.LeftType, 69);
                    Traverse_CarAgl(Scan_LineType.RightType, 69);
                    if (Car_RightLine.Agl_Row != 0 && Car_LeftLine.Agl_Row != 0)
                    {
                        CarTrailRow = (byte)((Car_RightLine.Agl_Row + Car_LeftLine.Agl_Row) / 2);
                        CarTrailLine = (byte)((Car_RightLine.Agl_Line + Car_LeftLine.Agl_Line) / 2);
                    }
                    else if (Car_LeftLine.Agl_Row != 0)
                    {
                        CarTrailRow = (byte)((Car_RightMargin[Car_RightLine.PointCnt].row + Car_LeftLine.Agl_Row) / 2);
                        CarTrailLine = (byte)((Car_RightMargin[Car_RightLine.PointCnt].line + Car_LeftLine.Agl_Line) / 2);
                    }
                    else if (Car_RightLine.Agl_Row != 0)
                    {
                        CarTrailRow = (byte)((Car_LeftMargin[Car_LeftLine.PointCnt].row + Car_RightLine.Agl_Row) / 2);
                        CarTrailLine = (byte)((Car_LeftMargin[Car_LeftLine.PointCnt].line + Car_RightLine.Agl_Line) / 2);
                    }
                    else
                    {
                        if (MeetingMode == 0)
                        {
                            CarTrailRow = 1;
                            CarTrailLine = 93;
                        }
                        else if(MeetingMode==1)
                        {
                            if(Car_LeftMargin[0].row == 0&& Car_RightMargin[0].row == 0)
                            {
                                CarTrailRow = 1;
                                CarTrailLine = 93;
                            }
                            else
                            {

                            }
                        }
                    }
                    break;
                case Meeting_st.Meeting_3:
                    break;
                case Meeting_st.Meeting_4:
                    EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                    EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                    break;
            }
            SetText_1("Trail Row = " + CarTrailRow + " Trail Line " + CarTrailLine);
            SetText_1("Meeting_State = " + Meeting_State);
            setText用户自定义("Meeting_State = " + Meeting_State);
            SetText_1("****************************Meeting_Scan End*************************");
            SetText_1(" ");
        }
        void Garage_Scan() // 入车库的扫描处理
        {
            SetText_1("****************************Garage_Scan Start*************************");
            GarageRow = GarageLine = 0;
            if (RoadType == RoadTypeEnum.Garage_L)
            {
                if (Error_Flag == 0)
                {
                    if (Garage_State == Garage.Garage_2 && LeftMargin_1[0].row != 0 && RightMargin_1[0].row != 0)
                    {
                        Garage_State = Garage.Garage_3;
                    }
                }
                switch (Garage_State)
                {
                    case Garage.Garage_0:
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                        Traverse_Agl_1(Scan_LineType.LeftType, 65);
                        NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 150, (byte)(LeftLine_1.Agl_Row + 10), 69);
                        Scan_And_Traverse_Agl_2(Scan_LineType.LeftType, 0, 100);

                        GarageRow = Garage_Judge(0);
                        SetText_1("GarageRow = " + GarageRow);
                        SetText_1("GarageLine = " + GarageLine);

                        if (GarageRow <= 30 && GarageRow >= 10)
                        {
                            Garage_State = Garage.Garage_1;
                        }
                        break;
                    case Garage.Garage_1:
                        for (int m = 31; m > 0; m = m - 5)
                        {
                            if (Zebra_Seed_Find(Scan_LineType.RightType, Scan_Direction.Horizontal, (byte)m, 85, 155) == 1)
                            {
                                if (Zebra_RegionScanLine(Scan_LineType.RightType, 0, 50, 0) == 1)
                                {
                                    while (Zebra_Seed_Find(Scan_LineType.LeftType, Scan_Direction.Horizontal, (byte)(Zebra_Line.Agl_Row + 5), (byte)(Zebra_Line.Agl_Line + 1), (byte)(Zebra_Line.Agl_Line + 10)) == 1)
                                    {
                                        Zebra_RegionScanLine(Scan_LineType.LeftType, 0, 50, 1);
                                    }
                                    break;
                                }
                                else
                                {
                                    Zebra_Line.Agl_Line = 185;
                                    Zebra_Line.Agl_Row = 0;
                                    //break;
                                }
                            }
                        }

                        if (Zebra_Line.Agl_Row != 0)
                        {
                            NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, (byte)(Zebra_Line.Agl_Line +1), (byte)(Zebra_Line.Agl_Row + 1), 60);
                            EightRegionScanLine_2(Scan_LineType.LeftType, 0, 120);
                        }
                        else
                        {
                            NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 110, 1, 60);
                            EightRegionScanLine_2(Scan_LineType.LeftType, 0, 120);
                        }
                        Traverse_Agl_2(Scan_LineType.LeftType, 65);
                        LeftLine_2.PointCnt = 0;

                        if (LeftLine_2.Agl_Row == 0)
                        {
                            Garage_State = Garage.Garage_2;
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, 1, 1, 93);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 60);
                            if (LeftMargin_1[0].row != 0)
                                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 30);
                        }
                        else if (LeftLine_2.Agl_Row != 0 && LeftLine_2.Agl_Row < 30)
                        {
                            NewSeedSet_2(Scan_LineType.RightType, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                            EightRegionScanLine_2(Scan_LineType.RightType, 0, 80);
                        }
                        break;
                    case Garage.Garage_2:
                        if (NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, 1, 1, 93) == 0)
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 20);
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 80);
                        if (NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 20) == 1)
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 30);
                        break;
                    case Garage.Garage_3:
                        if (NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, 1, 1, 93) == 0)
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50);
                        if (NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, 1, 93, 184) == 0)
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50);
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 40);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 40);
                        break;
                }
            }
            else
            {
                if (Error_Flag == 0)
                {
                    if (Garage_State == Garage.Garage_2 && LeftMargin_1[0].row != 0 && RightMargin_1[0].row != 0)
                    {
                        Garage_State = Garage.Garage_3;
                    }
                }
                switch (Garage_State)
                {
                    case Garage.Garage_0:
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                        Traverse_Agl_1(Scan_LineType.RightType, 65);
                        NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, RightLine_1.Agl_Line, (byte)(RightLine_1.Agl_Row + 10), 69);
                        Scan_And_Traverse_Agl_2(Scan_LineType.RightType, 0, 100);

                        Garage_Judge(1);
                        SetText_1("GarageRow = " + GarageRow);
                        SetText_1("GarageLine = " + GarageLine);

                        if (GarageRow <= 30 && GarageRow >= 10)
                        {
                            Garage_State = Garage.Garage_1;
                        }
                        break;
                    case Garage.Garage_1:
                        for (int m = 31; m > 0; m = m - 5)
                        {
                            if (Zebra_Seed_Find(Scan_LineType.LeftType, Scan_Direction.Horizontal, (byte)m, 30, 100) == 1)
                            {
                                if (Zebra_RegionScanLine(Scan_LineType.LeftType, 0, 50, 0) == 1)
                                {
                                    while (Zebra_Seed_Find(Scan_LineType.RightType, Scan_Direction.Horizontal, (byte)(Zebra_Line.Agl_Row + 5), (byte)(Zebra_Line.Agl_Line - 10), (byte)(Zebra_Line.Agl_Line - 1)) == 1)
                                    {
                                        Zebra_RegionScanLine(Scan_LineType.RightType, 0, 50, 1);
                                    }
                                    break;
                                }
                                else
                                {
                                    Zebra_Line.Agl_Line = 185;
                                    Zebra_Line.Agl_Row = 0;
                                    //break;
                                }
                            }
                        }
                        
                        if(Zebra_Line.Agl_Row!=0)
                        {
                            NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, (byte)(Zebra_Line.Agl_Line - 1), (byte)(Zebra_Line.Agl_Row + 1), 60);
                            EightRegionScanLine_2(Scan_LineType.RightType, 0, 120);
                        }
                        else
                        {
                            NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 75, 1, 60);
                            EightRegionScanLine_2(Scan_LineType.RightType, 0, 120);
                        }
                        Traverse_Agl_2(Scan_LineType.RightType, 65);
                        RightLine_2.PointCnt = 0;

                        if (RightLine_2.Agl_Row==0)
                        {
                            Garage_State = Garage.Garage_2;
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, 1, 93,184);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 60);
                            if(RightMargin_1[0].row!=0)
                                EightRegionScanLine_1(Scan_LineType.RightType, 0, 30);
                        }
                        else if(RightLine_2.Agl_Row != 0&& RightLine_2.Agl_Row <30)
                        {
                            NewSeedSet_2(Scan_LineType.LeftType, RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                            EightRegionScanLine_2(Scan_LineType.LeftType, 0, 80);
                        }
                        break;
                    case Garage.Garage_2:
                        if (NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, 1, 93, 184) == 0)
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 20);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 80);
                        if (NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 20) == 1)
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 30);
                        break;
                    case Garage.Garage_3:
                        if (NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, 1, 1, 93) == 0)
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50);
                        if (NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, 1, 93, 184) == 0)
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50);
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 40);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 40);
                        break;
                }
            }
            SetText_1("Garage_State = " + Garage_State);
            setText用户自定义("Garage_State = " + Garage_State);
            SetText_1("****************************Garage_Scan End*************************");
            SetText_1(" ");
        }
        /*void Out_Garage_Scan()
        {
            SetText_1("****************************Garage_Scan Start*************************");
            if (RoadType == RoadTypeEnum.Out_Garage_L)
            {
                if (Error_Flag == 0)
                {
                    if (Out_Garage_State == Out_Garage.Out_Garage_0
                        && J_Pixels[1][184] == white && J_Pixels[2][184] == white && J_Pixels[3][184] == white && J_Pixels[4][184] == white && J_Pixels[5][184] == white && J_Pixels[6][184] == white && J_Pixels[7][184] == white && J_Pixels[8][184] == white && J_Pixels[9][184] == white && J_Pixels[10][184] == white && J_Pixels[11][184] == white && J_Pixels[12][184] == white && J_Pixels[13][184] == white && J_Pixels[14][184] == white
                        && J_Pixels[1][1] == white && J_Pixels[2][1] == white && J_Pixels[3][1] == white && J_Pixels[4][1] == white && J_Pixels[5][1] == white && J_Pixels[6][1] == white && J_Pixels[7][1] == white && J_Pixels[8][1] == white && J_Pixels[9][1] == white && J_Pixels[10][1] == white && J_Pixels[11][1] == white && J_Pixels[12][1] == white && J_Pixels[13][1] == white && J_Pixels[14][1] == white
                        )
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_1;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_1 
                        && NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50) == 1
                        )
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_2;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_2
                         && ((J_Pixels[1][184] == black && J_Pixels[2][184] == black && J_Pixels[3][184] == black&& J_Pixels[4][184] == black && J_Pixels[5][184]==black && J_Pixels[6][184] == black && J_Pixels[7][184] == black && J_Pixels[8][184] == black && J_Pixels[9][184] == black && J_Pixels[10][184] == black)
                         || (J_Pixels[1][1] == black && J_Pixels[2][1] == black && J_Pixels[3][1] == black && J_Pixels[4][1] == black && J_Pixels[5][1]== black && J_Pixels[6][1] == black && J_Pixels[7][1] == black && J_Pixels[8][1] == black && J_Pixels[9][1] == black && J_Pixels[10][1] == black)))
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_3;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_3
                          && J_Pixels[7][184] == black && J_Pixels[8][184] == black && J_Pixels[9][184] == black
                          && J_Pixels[7][1] == black && J_Pixels[8][1] == black && J_Pixels[9][1] == black)
                    {
                        RoadType = RoadTypeEnum.Common;
                        Out_Garage_State = Out_Garage.Out_Garage_0;
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        return;
                    }
                }
                switch (Out_Garage_State)
                {
                    case Out_Garage.Out_Garage_0:
                    case Out_Garage.Out_Garage_1:
                        break;
                    case Out_Garage.Out_Garage_2:
                        NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 170, 10, 69);
                        break;
                    case Out_Garage.Out_Garage_3:
                        if (RightMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        }
                        else
                        {
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                            if (RightMargin_1[RightLine_1.PointCnt].row==0)
                            {
                                NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50);
                                EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                            }
                        }

                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        }
                        else
                        {
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                            if (LeftMargin_1[LeftLine_1.PointCnt].row ==0)
                            {
                                NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50);
                                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                            }
                        }
                        break;
                }
            }
            else
            {
                if (Error_Flag == 0)
                {
                    if (Out_Garage_State == Out_Garage.Out_Garage_0
                        && J_Pixels[1][184] == white && J_Pixels[2][184] == white && J_Pixels[3][184] == white && J_Pixels[4][184] == white && J_Pixels[5][184] == white && J_Pixels[6][184] == white && J_Pixels[7][184] == white && J_Pixels[8][184] == white && J_Pixels[9][184] == white && J_Pixels[10][184] == white && J_Pixels[11][184] == white && J_Pixels[12][184] == white && J_Pixels[13][184] == white && J_Pixels[14][184] == white
                        && J_Pixels[1][1] == white && J_Pixels[2][1] == white && J_Pixels[3][1] == white && J_Pixels[4][1] == white && J_Pixels[5][1] == white && J_Pixels[6][1] == white && J_Pixels[7][1] == white && J_Pixels[8][1] == white && J_Pixels[9][1] == white && J_Pixels[10][1] == white && J_Pixels[11][1] == white && J_Pixels[12][1] == white && J_Pixels[13][1] == white && J_Pixels[14][1] == white
                        )
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_1;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_1
                        && NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50) == 1
                        )
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_2;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_2
                        && ((J_Pixels[1][184] == black && J_Pixels[2][184] == black && J_Pixels[3][184] == black && J_Pixels[4][184] == black && J_Pixels[5][184] == black && J_Pixels[6][184] == black && J_Pixels[7][184] == black && J_Pixels[8][184] == black && J_Pixels[9][184] == black && J_Pixels[10][184] == black)
                         || (J_Pixels[1][1] == black && J_Pixels[2][1] == black && J_Pixels[3][1] == black && J_Pixels[4][1] == black && J_Pixels[5][1] == black && J_Pixels[6][1] == black && J_Pixels[7][1] == black && J_Pixels[8][1] == black && J_Pixels[9][1] == black && J_Pixels[10][1] == black)))
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_3;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_3
                          && J_Pixels[7][184] == black && J_Pixels[8][184] == black && J_Pixels[9][184] == black
                          && J_Pixels[7][1] == black && J_Pixels[8][1] == black && J_Pixels[9][1] == black)
                    {
                        RoadType = RoadTypeEnum.Common;
                        Out_Garage_State = Out_Garage.Out_Garage_0;
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        return;
                    }
                }
                switch (Out_Garage_State)
                {
                    case Out_Garage.Out_Garage_0:
                    case Out_Garage.Out_Garage_1:
                        break;
                    case Out_Garage.Out_Garage_2:
                        NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 15, 10, 69);
                        break;
                    case Out_Garage.Out_Garage_3:
                        if (RightMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        }
                        else
                        {
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                            if (RightMargin_1[RightLine_1.PointCnt].row == 0)
                            {
                                NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50);
                                EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                            }
                        }

                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        }
                        else
                        {
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                            if (LeftMargin_1[LeftLine_1.PointCnt].row == 0)
                            {
                                NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50);
                                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                            }
                        }
                        break;
                }
            }
            setText用户自定义("Out_Garage_State = " + Out_Garage_State);
            SetText_1("Out_Garage_State = " + Out_Garage_State);
            SetText_1("****************************Garage_Scan Start*************************");
            SetText_1(" ");
        }*/
        void Out_Garage_Scan() // 出车库的扫描处理
        {
            SetText_1("****************************Garage_Scan Start*************************");
            if (RoadType == RoadTypeEnum.Out_Garage_L)
            {
                if (Error_Flag == 0)
                {
                    if (Out_Garage_State == Out_Garage.Out_Garage_1
                         && J_Pixels[1][1] == white && J_Pixels[2][1] == white && J_Pixels[3][1] == white && J_Pixels[4][1] == white && J_Pixels[5][1] == white && J_Pixels[6][1] == white && J_Pixels[7][1] == white && J_Pixels[8][1] == white && J_Pixels[9][1] == white && J_Pixels[10][1] == white && J_Pixels[11][1] == white && J_Pixels[12][1] == white && J_Pixels[13][1] == white && J_Pixels[14][1] == white
                        )
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_2;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_2
                         && ((J_Pixels[1][184] == black && J_Pixels[2][184] == black && J_Pixels[3][184] == black && J_Pixels[4][184] == black && J_Pixels[5][184] == black && J_Pixels[6][184] == black && J_Pixels[7][184] == black && J_Pixels[8][184] == black && J_Pixels[9][184] == black && J_Pixels[10][184] == black)
                         || (J_Pixels[1][1] == black && J_Pixels[2][1] == black && J_Pixels[3][1] == black && J_Pixels[4][1] == black && J_Pixels[5][1] == black && J_Pixels[6][1] == black && J_Pixels[7][1] == black && J_Pixels[8][1] == black && J_Pixels[9][1] == black && J_Pixels[10][1] == black)))
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_3;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_3
                          && J_Pixels[7][184] == black && J_Pixels[8][184] == black && J_Pixels[9][184] == black
                          && J_Pixels[7][1] == black && J_Pixels[8][1] == black && J_Pixels[9][1] == black)
                    {
                        RoadType = RoadTypeEnum.Common;
                        Out_Garage_State = Out_Garage.Out_Garage_0;
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        return;
                    }
                    SetText_1("Out_Garage_State = "+ Out_Garage_State);
                }
                switch (Out_Garage_State)
                {
                    case Out_Garage.Out_Garage_0:
                        NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, 1, 1, 86);
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 100);
                        Traverse_Agl_1(Scan_LineType.RightType, 50);
                        if (RightLine_1.Agl_Row != 0 && RightLine_1.Agl_Row < 15)
                        {
                            Out_Garage_State = Out_Garage.Out_Garage_1;
                            NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 180, 20, 69);
                        }
                        break;
                    case Out_Garage.Out_Garage_1:
                        NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 180, 10, 69);
                        break;
                    case Out_Garage.Out_Garage_2:
                        NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 85, 20, 69);
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        if (RightLine_1.PointCnt > 60 && RightMargin_1[RightLine_1.PointCnt].row > 40)
                        {
                            Traverse_Agl_1(Scan_LineType.RightType, 69);
                        }
                        NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 180, 10, 69);
                        break;
                    case Out_Garage.Out_Garage_3:
                        if (RightMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        }
                        else
                        {
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                            if (RightMargin_1[RightLine_1.PointCnt].row == 0)
                            {
                                NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, 3, 1, (byte)(RightMargin_1[RightLine_1.PointCnt].line - 1));
                                EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                            }
                        }

                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        }
                        else
                        {
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                            if (LeftMargin_1[LeftLine_1.PointCnt].row == 0)
                            {
                                NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, 3, (byte)(LeftMargin_1[LeftLine_1.PointCnt].line + 1), 184);
                                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                            }
                        }
                        break;
                }
            }
            else
            {
                if (Error_Flag == 0)
                {
                    if (Out_Garage_State == Out_Garage.Out_Garage_1
                         && J_Pixels[1][184] == white && J_Pixels[2][184] == white && J_Pixels[3][184] == white && J_Pixels[4][184] == white && J_Pixels[5][184] == white && J_Pixels[6][184] == white && J_Pixels[7][184] == white && J_Pixels[8][184] == white && J_Pixels[9][184] == white && J_Pixels[10][184] == white && J_Pixels[11][184] == white && J_Pixels[12][184] == white && J_Pixels[13][184] == white && J_Pixels[14][184] == white
                        )
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_2;
                    }
                    else 
                    if (Out_Garage_State == Out_Garage.Out_Garage_2
                        && ((J_Pixels[1][184] == black && J_Pixels[2][184] == black && J_Pixels[3][184] == black && J_Pixels[4][184] == black && J_Pixels[5][184] == black && J_Pixels[6][184] == black && J_Pixels[7][184] == black && J_Pixels[8][184] == black && J_Pixels[9][184] == black && J_Pixels[10][184] == black)
                         || (J_Pixels[1][1] == black && J_Pixels[2][1] == black && J_Pixels[3][1] == black && J_Pixels[4][1] == black && J_Pixels[5][1] == black && J_Pixels[6][1] == black && J_Pixels[7][1] == black && J_Pixels[8][1] == black && J_Pixels[9][1] == black && J_Pixels[10][1] == black)))
                    {
                        Out_Garage_State = Out_Garage.Out_Garage_3;
                    }
                    else if (Out_Garage_State == Out_Garage.Out_Garage_3
                          && J_Pixels[7][184] == black && J_Pixels[8][184] == black && J_Pixels[9][184] == black
                          && J_Pixels[7][1] == black && J_Pixels[8][1] == black && J_Pixels[9][1] == black)
                    {
                        RoadType = RoadTypeEnum.Common;
                        Out_Garage_State = Out_Garage.Out_Garage_0;
                        EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        return;
                    }
                }
                switch (Out_Garage_State)
                {
                    case Out_Garage.Out_Garage_0:
                        NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, 1, 100, 184);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 100);
                        Traverse_Agl_1(Scan_LineType.LeftType, 50);
                        if (LeftLine_1.Agl_Row != 0 && LeftLine_1.Agl_Row < 15)
                        {
                            Out_Garage_State = Out_Garage.Out_Garage_1;
                            NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 5, 20, 69);
                        }
                        break;
                    case Out_Garage.Out_Garage_1:
                        NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 5, 20, 69);
                        break;
                    case Out_Garage.Out_Garage_2:
                        NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 100, 20, 69);
                        EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        if(LeftLine_1.PointCnt>60&&LeftMargin_1[LeftLine_1.PointCnt].row>40)
                        {
                            Traverse_Agl_1(Scan_LineType.LeftType, 69);
                        }
                        NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 5, 20, 69);
                        break;
                    case Out_Garage.Out_Garage_3:
                        if (RightMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 50);
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                        }
                        else
                        {
                            EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                            if (RightMargin_1[RightLine_1.PointCnt].row == 0)
                            {
                                NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, 3, 1, (byte)(RightMargin_1[RightLine_1.PointCnt].line - 1));
                                EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);
                            }
                        }
                        
                        if (LeftMargin_1[0].row == 0)
                        {
                            NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 50);
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                        }
                        else
                        {
                            EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                            if (LeftMargin_1[LeftLine_1.PointCnt].row == 0)
                            {
                                NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, 3, (byte)(LeftMargin_1[LeftLine_1.PointCnt].line+1), 184);
                                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                            }
                        }
                        break;
                }
            }
            setText用户自定义("Out_Garage_State = " + Out_Garage_State);
            SetText_1("Out_Garage_State = " + Out_Garage_State);
            SetText_1("****************************Garage_Scan Start*************************");
            SetText_1(" ");
        }
        void Out_Slope_Scan() // 出坡的扫描处理
        {
            SetText_1("****************************Out Slope Scan Start*************************");
            if (LeftMargin_1[0].row != 0 && RightMargin_1[0].row != 0)
            {
                SetText_1("Both Seed Find");
                LeftLine_1.RecStrError = MarginStrErrorCale_1(0, LeftLine_1.PointCnt, Scan_LineType.LeftType);
                RightLine_1.RecStrError = MarginStrErrorCale_1(0, RightLine_1.PointCnt, Scan_LineType.RightType);
                SetText_2("Slope Out LeftLine 1 RecStrLine = "+ LeftLine_1.RecStrError);
                SetText_2("Slope Out RightLine 1 RecStrLine = " + RightLine_1.RecStrError);
                if (LeftLine_1.RecStrError+ RightLine_1.RecStrError>3)
                {
                    RoadType = RoadTypeEnum.Turn_L;
                }
                else
                {
                    if (Error_Flag == 0)
                        RoadType = RoadTypeEnum.Common;
                }
            }
            if (LeftMargin_1[0].row != 0 && RightMargin_1[0].row == 0)
            {
                SetText_1("Right Seed Lost");
                LeftLine_1.RecStrError = MarginStrErrorCale_1(0, LeftLine_1.PointCnt, Scan_LineType.LeftType);
                SetText_2("Slope Out LeftLine 1 RecStrLine = " + LeftLine_1.RecStrError);
                if (LeftLine_1.RecStrError>1.5)
                {
                    RoadType = RoadTypeEnum.Turn_R;
                }
                else
                {
                    if (Error_Flag == 0)
                        RoadType = RoadTypeEnum.Common;
                }
            }
            else if (RightMargin_1[0].row != 0 && LeftMargin_1[0].row == 0)
            {
                SetText_1("Left Seed Lost");
                RightLine_1.RecStrError = MarginStrErrorCale_1(0, RightLine_1.PointCnt, Scan_LineType.RightType);
                SetText_2("Slope Out RightLine 1 RecStrLine = " + RightLine_1.RecStrError);
                if (RightLine_1.RecStrError>1.5)
                {
                    RoadType = RoadTypeEnum.Turn_L;
                }
                else
                {
                    if (Error_Flag == 0)
                        RoadType = RoadTypeEnum.Common;
                }
            }
            else if (LeftMargin_1[0].row == 0 && RightMargin_1[0].row == 0)
            {
                SetText_1("Both Seed Lost");
            }
            SetText_1("****************************Out Slope Scan End*************************");
        }
        void Slope_Scan() // 入坡的扫描处理
        {
            SetText_1("****************************Slope_Scan Start*************************");
            if (RoadType == RoadTypeEnum.Slope)
            {

                if (LeftMargin_1[0].row == 0)
                {
                    NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 1, 40);
                    EightRegionScanLine_1(Scan_LineType.LeftType, 0, 60);
                }
                else
                {
                    EightRegionScanLine_1(Scan_LineType.LeftType, 0, 60);
                    if (LeftLine_1.PointCnt < 20 && LeftMargin_1[LeftLine_1.PointCnt].row == 0)
                    {
                        for(byte x = 1;x<16;x++)
                        {
                            if (NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Horizontal, x, 120, 184) == 1)
                            {
                                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 60);
                                break;
                            }
                        }
                    }
                }
                if (RightMargin_1[0].row == 0)
                {
                    NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 1, 40);
                    EightRegionScanLine_1(Scan_LineType.RightType, 0, 60);
                }
                else
                {
                    EightRegionScanLine_1(Scan_LineType.RightType, 0, 60);
                    if (RightLine_1.PointCnt < 20 && RightMargin_1[RightLine_1.PointCnt].row == 0)
                    {
                        for(byte x= 1;x<16;x++)
                        {
                            if(NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Horizontal, x, 1, 66)==1)
                            {
                                EightRegionScanLine_1(Scan_LineType.RightType, 0, 60);
                                break;
                            }
                        }
                    }
                }
                switch (Slope_State)
                {
                    case Slope_st.Slope_0:
                        break;
                    case Slope_st.Slope_1:
                        break;
                    case Slope_st.Slope_2:
                        break;
                    case Slope_st.Slope_3:
                        break;
                    case Slope_st.Slope_4:
                        break;
                }
            }
            SetText_1("****************************Slope_Scan End*************************");
        }  
        void Common_Scan() // 普通赛道的常规处理
        {
            SetText_1("****************************Common_Scan Start*************************");
            if (LeftMargin_1[0].row == 0)
            {
                NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 184, 3, 35);
                EightRegionScanLine_1(Scan_LineType.LeftType, 0, 178);
            }
            else
            {
                EightRegionScanLine_1(Scan_LineType.LeftType, (byte)(LeftLine_1.PointCnt - 1), 178);
            }
            if (RightMargin_1[0].row == 0)
            {
                NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 1, 3, 35);
                EightRegionScanLine_1(Scan_LineType.RightType, 0, 178);
            }
            else
            {
                EightRegionScanLine_1(Scan_LineType.RightType, (byte)(RightLine_1.PointCnt - 1), 178);
            }
            Traverse_Turn_Agl_1(Scan_LineType.LeftType, 69);
            Traverse_Turn_Agl_1(Scan_LineType.RightType, 69);

            if (((LeftLine_1.Agl_Row != 0 && LeftLine_1.Agl_2_Row != 0) || (RightLine_1.Agl_Row != 0 && RightLine_1.Agl_2_Row != 0))
                && LeftLine_1.Agl_Row != 0 && RightLine_1.Agl_Row != 0)
            {
                CommonType = CommonTypeEnum.Small_Turn;
            }
            else 
            {
                CommonType = CommonTypeEnum.Short_Straight;
            }
            setText用户自定义("CommonType = "+ CommonType);
            SetText_1("****************************Common_Scan End*************************");
        }
        void Road_Judge() // 八领域扫描并且判断赛道类型
        {
            SetText_1(" ");
            SetText_1("****************************Road Judge Start*************************");
            if (RoadType != RoadTypeEnum.Slope
                && RoadType != RoadTypeEnum.Circle_L && RoadType != RoadTypeEnum.Circle_R
                && RoadType != RoadTypeEnum.Garage_L && RoadType != RoadTypeEnum.Garage_R
                && RoadType != RoadTypeEnum.Out_Garage_L && RoadType != RoadTypeEnum.Out_Garage_R
                && RoadType != RoadTypeEnum.Meeting)
            {
                if (LeftMargin_1[0].row != 0 && RightMargin_1[0].row != 0)
                {
                    SetText_1("Both Seed Find");
                    EightRegionScanLine_1(Scan_LineType.LeftType, 0, 120);
                    EightRegionScanLine_1(Scan_LineType.RightType, 0, 120);

                    Traverse_Agl_1(Scan_LineType.LeftType, 60);
                    LeftLine_1.AglStrError = MarginStrErrorCale_1(0, LeftLine_1.Agl_PointNum, Scan_LineType.LeftType);
                    L_front_line_error = LeftLine_1.AglStrError;
                    L_front_line_k = LeftLine_1.Line_k;
                    LeftLine_1.RecStrError = MarginStrErrorCale_1(0, LeftLine_1.Rec_Point, Scan_LineType.LeftType);

                    Traverse_Agl_1(Scan_LineType.RightType, 60);
                    RightLine_1.AglStrError = MarginStrErrorCale_1(0, RightLine_1.Agl_PointNum, Scan_LineType.RightType);
                    R_front_line_error = RightLine_1.AglStrError;
                    R_front_line_k = RightLine_1.Line_k;
                    RightLine_1.RecStrError = MarginStrErrorCale_1(0, RightLine_1.Rec_Point, Scan_LineType.RightType);

                    SetText_2("MarLeftLine_1.RecStrError = " + LeftLine_1.RecStrError);
                    SetText_2("MarLeftLine_1.AglStrError = " + LeftLine_1.AglStrError);
                    SetText_2("MarRightLine_1.RecStrError = " + RightLine_1.RecStrError);
                    SetText_2("MarRightLine_1.AglStrError = " + RightLine_1.AglStrError);
                    SetText_2("MarLeftLine_1.Line_k = " + LeftLine_1.Line_k);
                    SetText_2("MarRightLine_1.Line_k = " + RightLine_1.Line_k);
                    setText用户自定义("Error Sum = " + (LeftLine_1.RecStrError + RightLine_1.RecStrError));
                    if ((LeftLine_1.BroadWire_Cnt >= 10 || LeftLine_1.PointCnt < 50)
                        && LeftLine_1.Turn_Cnt < 50
                        && LeftLine_1.Agl_Row != 0 && LeftLine_1.Agl_Row <= 45
                        && ((RightLine_1.RecStrError < 2 && RightLine_1.EndLine >= 55) || (Error_Flag == 1 && RightLine_1.Agl_Row != 0 && RightLine_1.AglStrError < 2) || (RightLine_1.Agl_Row > LeftLine_1.Agl_Row && RightLine_1.AglStrError < 1))
                        && Garage_Judge(0) >= 1)
                    {
                        RoadType = RoadTypeEnum.Garage_L;
                        Garage_State = Garage.Garage_0;
                    }
                    else if ((LeftLine_1.BroadWire_Cnt >= 30 || (LeftLine_1.PointCnt < 60 && LeftLine_1.Error == 0))
                        //&& LeftLine_1.Turn_Cnt < 20
                        && LeftLine_1.AglStrError < 1.5
                        && LeftLine_1.EndLine < 50
                        && LeftLine_1.Agl_Row != 0
                        && RightLine_1.EndLine >= 55
                        && RightLine_1.RecStrError < 2
                        //|| (Error_Flag == 1 && RightLine_1.Agl_Row != 0 && RightLine_1.AglStrError < 2)
                        )
                    {
                        RoadType = RoadTypeEnum.Circle_L;
                        Circle_State = Circle.Circle_0;
                        Circle_cnt++;
                        AllSpecRoadCnt++;
                        //if (All_Circle_cnt == Circle_cnt++)
                        //    Circle_cnt = 0;
                    }
                    else if ((RightLine_1.BroadWire_Cnt >= 10 || RightLine_1.PointCnt < 50)
                        && RightLine_1.Turn_Cnt < 50
                        && RightLine_1.Agl_Row != 0 && RightLine_1.Agl_Row <= 45
                        && ((LeftLine_1.EndLine >= 55 && LeftLine_1.RecStrError < 2) || (Error_Flag == 1 && LeftLine_1.Agl_Row != 0 && LeftLine_1.AglStrError < 2) || (LeftLine_1.Agl_Row > RightLine_1.Agl_Row && LeftLine_1.AglStrError < 1))
                        && Garage_Judge(1) >= 1)
                    {
                        RoadType = RoadTypeEnum.Garage_R;
                        Garage_State = Garage.Garage_0;
                    }
                    else if ((RightLine_1.BroadWire_Cnt >= 30 || (RightLine_1.PointCnt < 60 && RightLine_1.Error == 0))
                        //&& RightLine_1.Turn_Cnt < 20
                        && RightLine_1.AglStrError < 1.5
                        && RightLine_1.EndLine < 50
                        && RightLine_1.Agl_Row != 0
                        && LeftLine_1.EndLine >= 55
                        && LeftLine_1.RecStrError < 2
                        //|| (Error_Flag == 1 && LeftLine_1.Agl_Row != 0 && LeftLine_1.AglStrError < 2)
                        )
                    {
                        RoadType = RoadTypeEnum.Circle_R;
                        Circle_State = Circle.Circle_0;
                        Circle_cnt++;
                        AllSpecRoadCnt++;
                        //if (All_Circle_cnt == Circle_cnt++)
                        //    Circle_cnt = 0;
                    }
                    else if (Error_Flag == 0
                          && (LeftLine_1.Agl_Row != 0 && RightLine_1.Agl_Row != 0)
                          && (LeftLine_1.BroadWire_Cnt >= 20 || LeftLine_1.PointCnt < 60)
                          && (RightLine_1.BroadWire_Cnt >= 20 || RightLine_1.PointCnt < 60)
                          && LeftLine_1.AglStrError < 1.8
                          && RightLine_1.AglStrError < 1.8
                          || (RoadType == RoadTypeEnum.CrossLine && (RightLine_1.PointCnt < 40 || LeftLine_1.PointCnt < 40))
                          )
                    {
                        RoadType = RoadTypeEnum.CrossLine;
                    }
                    else if (
                        (((LeftLine_1.Line_k < -1.5 && (RightLine_1.Line_k < 0.2 || LeftLine_1.RecStrError + RightLine_1.RecStrError >= 16)) || LeftLine_1.Line_k < -2) && RightLine_1.Agl_Row < 45)
                        || ((RoadType == RoadTypeEnum.Turn_L || RoadType == RoadTypeEnum.Turn_R) && LeftLine_1.Line_k < -1.5)
                        )
                    {
                        if (RightLine_1.Agl_Row != 0)
                        {
                            SetText_1("//Turn Judge Start");
                            R_forward_line_error = MarginStrErrorCale_1(RightLine_1.Agl_PointNum, RightLine_1.Rec_Point, Scan_LineType.RightType);
                            R_forwad_line_k = RightLine_1.Line_k;
                            if (my_fabs(R_front_line_k - R_forwad_line_k) >= 7
                                && RightLine_1.Agl_Row < 60)
                            {
                                RoadType = RoadTypeEnum.Slow_Turn_R;
                            }
                            else
                            {
                                RoadType = RoadTypeEnum.Turn_R;
                            }
                            //setText用户自定义("front_line_k = " + front_line_k + " forwad_line_k = " + forwad_line_k);
                            //setText用户自定义("front_line_error = " + front_line_error + " forward_line_error = " + forward_line_error);
                            SetText_1("//Turn Judge End");
                        }
                        else
                            RoadType = RoadTypeEnum.Turn_R;
                    }
                    else if (
                        (((RightLine_1.Line_k > 1.5 && (LeftLine_1.Line_k > -0.2 || LeftLine_1.RecStrError + RightLine_1.RecStrError >= 16)) || RightLine_1.Line_k > 2) && LeftLine_1.Agl_Row < 45)
                        ||((RoadType==RoadTypeEnum.Turn_L|| RoadType == RoadTypeEnum.Turn_R)&& RightLine_1.Line_k > 1.3)
                        )
                    {
                        if (LeftLine_1.Agl_Row != 0)
                        {
                            SetText_1("//Turn Judge Start");
                            L_forward_line_error = MarginStrErrorCale_1(LeftLine_1.Agl_PointNum, LeftLine_1.Rec_Point, Scan_LineType.LeftType);
                            L_forwad_line_k = LeftLine_1.Line_k;
                            if (my_fabs(L_front_line_k - L_forwad_line_k) >= 7
                                && LeftLine_1.Agl_Row < 60)
                            {
                                RoadType = RoadTypeEnum.Slow_Turn_L;
                            }
                            else
                            {
                                RoadType = RoadTypeEnum.Turn_L;
                            }
                            //setText用户自定义("front_line_k = " + front_line_k + " forwad_line_k = " + forwad_line_k);
                            //setText用户自定义("front_line_error = " + front_line_error + " forward_line_error = " + forward_line_error);
                            SetText_1("//Turn Judge End");
                        }
                        else
                            RoadType = RoadTypeEnum.Turn_L;
                    }
                    else
                    {
                        if (Error_Flag == 0 || (Error_Flag == 1 && RoadType != RoadTypeEnum.Turn_L && RoadType != RoadTypeEnum.Turn_R))
                            RoadType = RoadTypeEnum.Common;
                    }
                }
                if (LeftMargin_1[0].row != 0 && RightMargin_1[0].row == 0 && RoadType != RoadTypeEnum.CrossLine)
                {
                    SetText_1("Right Seed Lost");
                    EightRegionScanLine_1(Scan_LineType.LeftType, 0, 148);
                    Traverse_Agl_1(Scan_LineType.LeftType, 60);
                    LeftLine_1.AglStrError = MarginStrErrorCale_1(0, LeftLine_1.Agl_PointNum, Scan_LineType.LeftType);
                    LeftLine_1.RecStrError = MarginStrErrorCale_1(0, LeftLine_1.Rec_Point, Scan_LineType.LeftType);
                    SetText_2("MarLeftLine_1.RecStrError = " + LeftLine_1.RecStrError);
                    SetText_2("MarLeftLine_1.AglStrError = " + LeftLine_1.AglStrError);
                    SetText_2("MarLeftLine_1.Line_k = " + LeftLine_1.Line_k);
                    if (LeftLine_1.Agl_Row != 0 && LeftLine_1.Error == 0)
                    {
                        if (NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 5, LeftLine_1.Agl_Row, 60) == 1
                            && Scan_And_Traverse_Agl_2(Scan_LineType.RightType, 0, 120) == 1)
                        {
                            if ((LeftLine_1.BroadWire_Cnt > 20 || LeftLine_1.PointCnt < 35)
                                    && LeftLine_1.Agl_Row != 0
                                    && LeftLine_1.AglStrError < 1.5
                                    && RightLine_2.Agl_Row != 0
                                    && NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, LeftLine_1.Agl_Line, (byte)(LeftLine_1.Agl_Row + 1), (byte)(RightLine_2.Agl_Row + 20)) == 1
                                    && Scan_And_Traverse_Agl_2(Scan_LineType.LeftType, 0, 120) == 1
                                    )
                            {
                                RoadType = RoadTypeEnum.CrossLine;
                            }
                        }
                        /*else if (LeftLine_1.Line_k < -1.6)
                        {
                            RoadType = RoadTypeEnum.Turn_R;
                        }*/
                    }
                    else if ((LeftLine_1.RecStrError < 1.5 || (LeftLine_1.Error == 1 && LeftLine_1.Agl_Row != 0 && LeftLine_1.AglStrError < 2 && my_fabs(LeftLine_1.Err_Cnt - LeftLine_1.Agl_PointNum) < 30))
                        && NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 5, 10, 55) == 1
                        && Scan_And_Traverse_Circle_Agl_2(Scan_LineType.RightType, 0, 120) == 1
                        && RightLine_2.Agl_Row<60)
                    {
                        R_front_line_error = MarginStrErrorCale_2(0, RightLine_2.Agl_PointNum, Scan_LineType.RightType);
                        R_front_line_k = RightLine_2.Line_k;
                        R_forward_line_error = MarginStrErrorCale_2(RightLine_2.Agl_PointNum, (byte)(RightLine_2.Agl_PointNum + 20 < RightLine_2.PointCnt ? RightLine_2.Agl_PointNum + 20 : RightLine_2.PointCnt), Scan_LineType.RightType);
                        R_forwad_line_k = RightLine_2.Line_k;
                        SetText_2("R_front_line_k = " + R_front_line_k + " R_front_line_error = " + R_front_line_error);
                        SetText_2("R_forwad_line_k = " + R_forwad_line_k + " R_forward_line_error = " + R_forward_line_error);
                        if (my_fabs(R_front_line_k - R_forwad_line_k) > 4
                        && R_front_line_k > 2
                        && R_front_line_error >= 2
                        && R_forwad_line_k < -1.5
                        && LeftLine_1.EndLine >= 55
                        && LeftLine_1.Line_k > -1.5
                        )
                        {
                            if (Garage_Judge(1) >= 1)
                            {
                                RoadType = RoadTypeEnum.Garage_R;
                                Garage_State = Garage.Garage_0;
                            }
                            else
                            {
                                RoadType = RoadTypeEnum.Circle_R;
                                Circle_State = Circle.Circle_0;
                                Circle_cnt++;
                                AllSpecRoadCnt++;
                                //if (All_Circle_cnt == Circle_cnt++)
                                //    Circle_cnt = 0;
                            }
                        }
                    }
                    else
                    {
                        if (NewSeedFind_1(Scan_LineType.RightType, Scan_Direction.Vertical, 5, 4, (byte)(LeftLine_1.EndLine - 3)) == 1)
                        {
                            SetText_1("Lost Left Agl,Common Scan");
                            Scan_And_Traverse_Agl_1(Scan_LineType.RightType, 0, 120);
                            if (
                                //LeftLine_1.Line_k < -1
                           (LeftLine_1.Line_k < -1.4 && RightLine_1.Line_k < 0.2)
                        || (LeftLine_1.Line_k < -2.0)
                        )
                            {
                                RoadType = RoadTypeEnum.Turn_R;
                            }
                            else
                            {
                                if (Error_Flag == 0)
                                    RoadType = RoadTypeEnum.Common;
                            }
                        }
                        else
                        {
                            SetText_1("No Right 1 Region");
                            RoadType = RoadTypeEnum.Turn_R;
                        }

                    }
                }
                else if (RightMargin_1[0].row != 0 && LeftMargin_1[0].row == 0 && RoadType != RoadTypeEnum.CrossLine)
                {
                    SetText_1("Left Seed Lost");
                    EightRegionScanLine_1(Scan_LineType.RightType, 0, 148);
                    Traverse_Agl_1(Scan_LineType.RightType, 60);
                    RightLine_1.AglStrError = MarginStrErrorCale_1(0, RightLine_1.Agl_PointNum, Scan_LineType.RightType);
                    RightLine_1.RecStrError = MarginStrErrorCale_1(0, RightLine_1.Rec_Point, Scan_LineType.RightType);
                    SetText_2("MarRightLine_1.RecStrError = " + RightLine_1.RecStrError);
                    SetText_2("MarRightLine_1.AglStrError = " + RightLine_1.AglStrError);
                    SetText_2("MarRightLine_1.Line_k = " + RightLine_1.Line_k);
                    if (RightLine_1.Agl_Row != 0 && RightLine_1.Error == 0)
                    {
                        if (NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 180, RightLine_1.Agl_Row, 60) == 1
                            && Scan_And_Traverse_Agl_2(Scan_LineType.LeftType, 0, 120) == 1)
                        {
                            if ((RightLine_1.BroadWire_Cnt > 20 || RightLine_1.PointCnt < 35)
                                && RightLine_1.Agl_Row != 0
                                && RightLine_1.AglStrError < 1.5
                                && LeftLine_2.Agl_Row != 0
                                && NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, RightLine_1.Agl_Line, (byte)(RightLine_1.Agl_Row + 1), (byte)(LeftLine_2.Agl_Row + 20)) == 1
                                && Scan_And_Traverse_Agl_2(Scan_LineType.RightType, 0, 120) == 1
                                )
                            {
                                RoadType = RoadTypeEnum.CrossLine;
                            }
                        }
                        /*else if (RightLine_1.Line_k > 1.6)
                        {
                            RoadType = RoadTypeEnum.Turn_L;
                        }*/
                    }
                    else if ((RightLine_1.RecStrError < 1.5 || (RightLine_1.Error == 1 && RightLine_1.Agl_Row != 0 && RightLine_1.AglStrError < 2 && my_fabs(RightLine_1.Err_Cnt - RightLine_1.Agl_PointNum) < 30))
                        && NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 180, 10, 55) == 1
                        && Scan_And_Traverse_Circle_Agl_2(Scan_LineType.LeftType, 0, 120) == 1
                        && LeftLine_2.Agl_Row < 60)
                    {
                        L_front_line_error = MarginStrErrorCale_2(0, LeftLine_2.Agl_PointNum, Scan_LineType.LeftType);
                        L_front_line_k = LeftLine_2.Line_k;
                        L_forward_line_error = MarginStrErrorCale_2(LeftLine_2.Agl_PointNum, (byte)(LeftLine_2.Agl_PointNum + 20 < LeftLine_2.PointCnt ? LeftLine_2.Agl_PointNum + 20 : LeftLine_2.PointCnt), Scan_LineType.LeftType);
                        L_forwad_line_k = LeftLine_2.Line_k;
                        SetText_2("L_front_line_k = " + L_front_line_k + " L_front_line_error = " + L_front_line_error);
                        SetText_2("L_forwad_line_k = " + L_forwad_line_k + " L_forward_line_error = " + L_forward_line_error);
                        if (my_fabs(L_front_line_k - L_forwad_line_k) > 4
                            && L_front_line_k < -2
                            && L_front_line_error >= 2
                            && L_forwad_line_k > 1.5
                            && RightLine_1.EndLine >= 55
                            && RightLine_1.Line_k < 1.5
                            )
                        {
                            if (Garage_Judge(0) >= 1)
                            {
                                RoadType = RoadTypeEnum.Garage_L;
                                Garage_State = Garage.Garage_0;
                            }
                            else
                            {
                                RoadType = RoadTypeEnum.Circle_L;
                                Circle_State = Circle.Circle_0;
                                Circle_cnt++;
                                AllSpecRoadCnt++;
                                //if (All_Circle_cnt == Circle_cnt++)
                                //    Circle_cnt = 0;
                            }
                        }
                    }
                    else
                    {
                        if (NewSeedFind_1(Scan_LineType.LeftType, Scan_Direction.Vertical, 180, 4, (byte)(RightLine_1.EndLine - 3)) == 1)
                        {
                            SetText_1("Lost Right Agl,Common Scan");
                            Scan_And_Traverse_Agl_1(Scan_LineType.LeftType, 0, 120);
                            LeftLine_1.RecStrError = MarginStrErrorCale_1(0, LeftLine_1.Rec_Point, Scan_LineType.LeftType);
                            if (
                                //RightLine_1.Line_k > 1
                               (LeftLine_1.Line_k > -0.2 && RightLine_1.Line_k > 1.4)
                            || (RightLine_1.Line_k > 2.0)
                            )
                            {
                                RoadType = RoadTypeEnum.Turn_L;
                            }
                            else
                            {
                                if (Error_Flag == 0)
                                    RoadType = RoadTypeEnum.Common;
                            }
                        }
                        else
                        {
                            SetText_1("No Left 1 Region");
                            RoadType = RoadTypeEnum.Turn_L;
                        }
                    }
                }
                else if (LeftMargin_1[0].row == 0 && RightMargin_1[0].row == 0 && RoadType != RoadTypeEnum.CrossLine)
                {
                    SetText_1("Both Seed Lost");
                    NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 165, 1, 69);
                    EightRegionScanLine_2(Scan_LineType.LeftType, 0, 100);
                    NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 20, 1, 69);
                    EightRegionScanLine_2(Scan_LineType.RightType, 0, 100);
                    Traverse_Agl_2(Scan_LineType.LeftType, 65);
                    //LeftLine_2.AglStrError = MarginStrErrorCale_2(0, LeftLine_2.Agl_PointNum, Scan_LineType.LeftType);
                    //LeftLine_2.RecStrError = MarginStrErrorCale_2(0, LeftLine_2.Rec_Point, Scan_LineType.LeftType);
                    Traverse_Agl_2(Scan_LineType.RightType, 65);
                    //RightLine_2.AglStrError = MarginStrErrorCale_2(0, RightLine_2.Agl_PointNum, Scan_LineType.RightType);
                    //RightLine_2.RecStrError = MarginStrErrorCale_2(0, RightLine_2.Rec_Point, Scan_LineType.RightType);
                    SetText_2("MarLeftLine_2.RecStrError = " + LeftLine_2.RecStrError);
                    SetText_2("MarRightLine_2.RecStrError = " + RightLine_2.RecStrError);
                    SetText_1("Skip Judge");
                    SetText_1(" ");
                }
            }
            else
            {
                SetText_1("Special Road! Skip Judge!");
            }
            SetText_1("RoadType = " + RoadType);
            SetText_1("****************************Road Judge End*************************");
            SetText_1(" ");
        }
        void Road_Scan() // 根据不同的赛道类型调整扫描策略
        {
            if (RoadType == RoadTypeEnum.Circle_L || RoadType == RoadTypeEnum.Circle_R)
            {
                Circle_Scan();
            }
            else if (RoadType == RoadTypeEnum.Garage_L || RoadType == RoadTypeEnum.Garage_R)
            {
                Garage_Scan();
            }
            else if (RoadType == RoadTypeEnum.Out_Garage_L || RoadType == RoadTypeEnum.Out_Garage_R)
            {
                Out_Garage_Scan();
            }
            else if (RoadType == RoadTypeEnum.Meeting)
            {
                Meeting_Scan();
            }
            else if (RoadType == RoadTypeEnum.CrossLine)
            {
                CrossLine_Scan();
            }
            else if (RoadType == RoadTypeEnum.Turn_L || RoadType == RoadTypeEnum.Turn_R)
            {
                Turn_Scan();
            }
            else if (RoadType == RoadTypeEnum.Slope)
            {
                Slope_Scan();
            }
            else if(RoadType==RoadTypeEnum.Common)
            {
                Common_Scan();
            }
            else
            {

            }
            if (Slope_Fps > 0 && RoadType != RoadTypeEnum.Slope)
            {
                if (RoadType == RoadTypeEnum.Common || RoadType == RoadTypeEnum.Straight || RoadType == RoadTypeEnum.Turn_L || RoadType == RoadTypeEnum.Turn_R)
                {
                    setText用户自定义("Slope_Fps = " + Slope_Fps);
                    Out_Slope_Scan();
                }
                else
                    Slope_Fps = 0;
            }
        }
        void Type_Info() // 打印赛道情况
        {
            SetText_1("**************************** Summary *************************");
            SetText_1("Left_1 PointCnt:" + LeftLine_1.PointCnt + "           Right_1 PointCnt:" + RightLine_1.PointCnt);
            SetText_1("Left_1 BroadWire_Cnt:" + LeftLine_1.BroadWire_Cnt + "          Right_1 BroadWire_Cnt:" + RightLine_1.BroadWire_Cnt);
            SetText_1("Left_1 Straight_Cnt:" + LeftLine_1.Straight_Cnt + "            Right_1 Straight_Cnt:" + RightLine_1.Straight_Cnt);
            SetText_1("Left_1 Agl_PointNum:" + LeftLine_1.Agl_PointNum + "            Right_1 Agl_PointNum:" + RightLine_1.Agl_PointNum);
            SetText_1("Left_1 Turn_Cnt:" + LeftLine_1.Turn_Cnt + "            Right_1 Turn_Cnt:" + RightLine_1.Turn_Cnt);
            SetText_1("Left_1 Reserve_Cnt:" + LeftLine_1.Reserve_Cnt + "            Right_1 Reserve_Cnt:" + RightLine_1.Reserve_Cnt);
            SetText_1("Left_1 Rec_Point:" + LeftLine_1.Rec_Point + "            Right_1 Rec_Point:" + RightLine_1.Rec_Point);
            SetText_1("Left_1 StartLine:" + LeftLine_1.StartLine + "            Right_1 StartLine:" + RightLine_1.StartLine);
            SetText_1("Left_1 EndLine:" + LeftLine_1.EndLine + "            Right_1 EndLine:" + RightLine_1.EndLine);
            SetText_1("Left_2 PointCnt:" + LeftLine_2.PointCnt + "           Right_2 PointCnt:" + RightLine_2.PointCnt);
            SetText_1("Left_2 BroadWire_Cnt:" + LeftLine_2.BroadWire_Cnt + "          Right_2 BroadWire_Cnt:" + RightLine_2.BroadWire_Cnt);
            SetText_1("Left_2 Straight_Cnt:" + LeftLine_2.Straight_Cnt + "            Right_2 Straight_Cnt:" + RightLine_2.Straight_Cnt);
            SetText_1("Left_2 Turn_Cnt:" + LeftLine_2.Turn_Cnt + "            Right_2 Turn_Cnt:" + RightLine_2.Turn_Cnt);
            SetText_1("Left_2 Reserve_Cnt:" + LeftLine_2.Reserve_Cnt + "            Right_2 Reserve_Cnt:" + RightLine_2.Reserve_Cnt);
            SetText_1("Left_2 Rec_Point:" + LeftLine_2.Rec_Point + "            Right_2 Rec_Point:" + RightLine_2.Rec_Point);
            SetText_1("Left_2 StartLine:" + LeftLine_2.StartLine + "            Right_2 StartLine:" + RightLine_2.StartLine);
            SetText_1("Left_2 EndLine:" + LeftLine_2.EndLine + "            Right_2 EndLine:" + RightLine_2.EndLine);
            if (RoadType == RoadTypeEnum.Circle_L || RoadType == RoadTypeEnum.Circle_R)
                SetText_1("Circle_State = " + Circle_State);
            SetText_1("**************************** Summary *************************");
            SetText_1(" ");
        }
        void ScanLine() // 八邻域扫描、判断 总函数
        {
            SetText_1("InitSeedLine = "+ InitSeedLine);
            Initial_Seed_Find(Scan_LineType.LeftType, 15); // 八领域初始种子设置
            Initial_Seed_Find(Scan_LineType.RightType, 15); // 八邻域初始种子设置
            Road_Judge(); // 赛道类型判断
            Road_Scan();
#if UpperComputer
            Type_Info();
#endif
            if (RoadType != RoadTypeEnum.Meeting)
                Camera_Protect();
        }
        #endregion
        #region 从领域边线中提取赛道边线函数及变量
        byte Scan_EndRow;
        byte Left_Bound_StartLine;
        byte Left_Bound_EndLine;
        byte Right_Bound_StartLine;
        byte Right_Bound_EndLine;
        byte Set_Start_Edge(Scan_LineType Line_RorL, byte Num) // 设置对邻域边线的提取范围
        {
            byte i;
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Num == 1)
                {
                    //SetText_1("L1");
                    if (LeftLine_1.PointCnt == 0)
                    {
                        return 255;
                    }
                    for (i = 1; i < LeftLine_1.PointCnt; i++)
                    {
                        if (LeftMargin_1[i].row > LeftMargin_1[i - 1].row)
                        {
                            //SetText_1("i = " + i + " LeftMargin_1[" + (i - 1) + "].row " + LeftMargin_1[i - 1].row + " LeftMargin_1[" + (i - 1) + "].line " + LeftMargin_1[i - 1].line);
                            return (byte)(i - 1);
                        }
                    }
                }
                else
                {
                    //SetText_1("L2");
                    if (LeftLine_2.PointCnt == 0)
                    {
                        return 255;
                    }
                    else if (LeftLine_2.Agl_Row != 0)
                    {
                        //SetText_1("StartPoint = LeftLine_2.Agl_PointNum = "+ LeftLine_2.Agl_Row);
                        return LeftLine_2.Agl_PointNum;
                    }
                    for (i = 1; i < LeftLine_2.PointCnt; i++)
                    {
                        //SetText_1("i = " + i + " LeftMargin_2[" + (i - 1) + "].row " + LeftMargin_2[i - 1].row + " LeftMargin_2[" + (i - 1) + "].line " + LeftMargin_2[i - 1].line);
                        if (LeftMargin_2[i].row > LeftMargin_2[i - 1].row)
                        {
                            //SetText_1("i = " + i + " LeftMargin_2[" + (i - 1) + "].row " + LeftMargin_2[i - 1].row + " LeftMargin_2[" + (i - 1) + "].line " + LeftMargin_2[i - 1].line);
                            return (byte)(i - 1);
                        }
                    }
                }
            }
            else
            {
                if (Num == 1)
                {
                    //SetText_1("R1");
                    if (RightLine_1.PointCnt == 0)
                    {
                        return 255;
                    }
                    for (i = 1; i < RightLine_1.PointCnt; i++)
                    {
                        if (RightMargin_1[i].row > RightMargin_1[i - 1].row)
                        {
                            //SetText_1("i = " + i + " RightMargin_1[" + (i - 1) + "].row " + RightMargin_1[i - 1].row + " RightMargin_1[" + (i - 1) + "].line " + RightMargin_1[i - 1].line);
                            return (byte)(i - 1);
                        }
                    }
                }
                else
                {
                    //SetText_1("R2");
                    if (RightLine_2.PointCnt == 0)
                    {
                        return 255;
                    }
                    else if (RightLine_2.Agl_Row != 0)
                    {
                        return RightLine_2.Agl_PointNum;
                    }
                    for (i = 1; i < RightLine_2.PointCnt; i++)
                    {
                        if (RightMargin_2[i].row > RightMargin_2[i - 1].row)
                        {
                            //SetText_1("i = " + i + " RightMargin_2[" + (i - 1) + "].row " + RightMargin_2[i - 1].row + " RightMargin_2[" + (i - 1) + "].line " + RightMargin_2[i - 1].line);
                            return (byte)(i - 1);
                        }
                    }
                }
            }
            return 255;
        }
        void Find_Edge(Scan_LineType Line_RorL, byte Num) // 对领域边界提取赛道边线
        {
            byte i, n, StartPoint, EndRow;
            StartPoint = Set_Start_Edge(Line_RorL, Num);
            //SetText_1("StartPoint = "+ StartPoint);
            EndRow = 70;
            if (StartPoint == 255)
            {
                SetText_1("! No Start Point .Skip Find Edge " + Line_RorL + Num);
                if (Line_RorL == Scan_LineType.LeftType && Num == 1)
                    Left_Bound_EndLine = 1;
                else if (Line_RorL == Scan_LineType.RightType && Num == 1)
                    Right_Bound_EndLine = 1;
                return;
            }
            if (Line_RorL == Scan_LineType.LeftType)
            {
                if (Num == 1)
                {
                    n = StartPoint;
                    //EndRow = (byte)(LeftLine_1.Agl_Row == 0 ? 70 : LeftLine_1.Agl_Row);
                    for (i = LeftMargin_1[StartPoint].row; i < EndRow; i++)
                    {
                        while (LeftMargin_1[n].row < i && n < 179)
                        {
                            n++;
                            if (LeftMargin_1[n].row == 0 || n == LeftLine_1.PointCnt)
                            {
                                //SetText_1("End Right Margin: LeftMargin_1[" + n + "].row = " + LeftMargin_1[n].row + " LeftMargin_1[" + n + "].line = " + LeftMargin_1[n].line);
                                Left_Bound_EndLine = (byte)(i - 1);
                                return;
                            }
                        }
                        //SetText_1("L_black[" + i + "] = LeftMargin_1[" + n + "].line = " + LeftMargin_1[n].line + " LeftMargin_1[" + n + "].row = " + LeftMargin_1[n].row);
                        if (Left_Bound_StartLine == 0)
                        {
                            Left_Bound_StartLine = i;
                            //SetText_1("Left_Bound_StartLine = " + Left_Bound_StartLine);
                        }
                        L_black[i] = LeftMargin_1[n].line;
                    }
                    Left_Bound_EndLine = (byte)(EndRow - 1);
                }
                else
                {
                    //SetText_1("1");
                    n = StartPoint;
                    for (i = LeftMargin_2[StartPoint].row; i < EndRow; i++)
                    {
                        //SetText_1("LeftMargin_2["+StartPoint+"].row = "+ LeftMargin_2[StartPoint].row);
                        while (LeftMargin_2[n].row < i && n < 149)
                        {
                            n++;
                            //SetText_1("LeftMargin_2[" + n + "].line  = " + LeftMargin_2[ n ].line + " LeftMargin_2[" + n + "].row  = " +LeftMargin_2[n].row + " <  " + i);
                            if (LeftMargin_2[n].row == 0 || n == LeftLine_2.PointCnt)
                            {
                                //SetText_1("Left_Bound_StartLine = " + Left_Bound_StartLine);
                                Left_Bound_EndLine = (byte)(i - 1);
                                return;
                            }
                        }
                        //SetText_1("L_black["+i+"] = LeftMargin_2["+n+"].line = " + LeftMargin_2[n].line + " LeftMargin_2["+n+"].row = "+ LeftMargin_2[n].row);
                        if (Left_Bound_StartLine == 0)
                            Left_Bound_StartLine = i;
                        L_black[i] = LeftMargin_2[n].line;
                    }
                    Left_Bound_EndLine = (byte)(EndRow - 1);
                }
            }
            else
            {
                if (Num == 1)
                {
                    n = StartPoint;
                    //EndRow = (byte)(RightLine_1.Agl_Row == 0 ? 70 : RightLine_1.Agl_Row);
                    for (i = RightMargin_1[StartPoint].row; i < EndRow; i++)
                    {
                        while (RightMargin_1[n].row < i && n < 179)
                        {
                            n++;
                            if (RightMargin_1[n].row == 0 || n == RightLine_1.PointCnt)
                            {
                                Right_Bound_EndLine = (byte)(i - 1);
                                //SetText_1("End Right Margin: RightMargin_1["+n+"].row = "+ RightMargin_1[n].row + " RightMargin_1["+n+"].line = "+ RightMargin_1[n].line);
                                //SetText_1("Right_Bound_EndLine = "+ Right_Bound_EndLine);
                                return;
                            }
                        }
                        if (Right_Bound_StartLine == 0)
                            Right_Bound_StartLine = i;
                        R_black[i] = RightMargin_1[n].line;
                        //SetText_1("R_black["+i+"] = "+ "RightMargin_1["+n+"].line = " + RightMargin_1[n].line+ " RightMargin_1[" + n + "].row = " + RightMargin_1[n].row);
                    }
                    Right_Bound_EndLine = (byte)(EndRow - 1);
                }
                else
                {
                    n = StartPoint;
                    for (i = RightMargin_2[StartPoint].row; i < EndRow; i++)
                    {
                        while (RightMargin_2[n].row < i&&n<149)
                        {
                            n++;
                            if (RightMargin_2[n].row == 0 || n == RightLine_2.PointCnt)
                            {
                                Right_Bound_EndLine = (byte)(i - 1);
                                return;
                            }
                        }
                        if (Right_Bound_StartLine == 0)
                            Right_Bound_StartLine = i;
                        //SetText_1("Right_Bound_StartLine = "+ Right_Bound_StartLine);
                        R_black[i] = RightMargin_2[n].line;
                        //SetText_1("R_black[" + i + "] = RightMargin_2[" + n + "].line = " + RightMargin_2[n].line + "RightMargin_2[" + n + "].row = " + RightMargin_2[n].row);
                        //SetText_1("i = "+i);
                    }
                    Right_Bound_EndLine = (byte)(EndRow - 1);
                    SetText_1("Right_Bound_EndLine = " + Right_Bound_EndLine);
                }
            }
        }
        void Angley_Set() // 从邻域边线中提取拐点
        {
            if (LeftLine_1.Agl_Row != 0)
            {
                L_black[LeftLine_1.Agl_Row] = LeftLine_1.Agl_Line;
            }
            if (LeftLine_2.Agl_Row != 0 && LeftMargin_2[0].row <= LeftLine_2.Agl_Row)
            {
                L_black[LeftLine_2.Agl_Row] = LeftLine_2.Agl_Line;
            }
            if (RightLine_1.Agl_Row != 0)
            {
                R_black[RightLine_1.Agl_Row] = RightLine_1.Agl_Line;
            }
            if (RightLine_2.Agl_Row != 0 && RightMargin_2[0].row <= RightLine_2.Agl_Row)
            {
                R_black[RightLine_2.Agl_Row] = RightLine_2.Agl_Line;
            }
        }
        void Exract_Edge() // 提取赛道边线 总函数
        {
            Left_Bound_StartLine = 0;
            Left_Bound_EndLine = 69;
            Right_Bound_StartLine = 0;
            Right_Bound_EndLine = 69;
            SetText_1("****************************Exract Edge Start*************************");
            Find_Edge(Scan_LineType.LeftType, 1);
            Find_Edge(Scan_LineType.LeftType, 2);
            Find_Edge(Scan_LineType.RightType, 1);
            Find_Edge(Scan_LineType.RightType, 2);
            Angley_Set();
            Scan_EndRow = Left_Bound_EndLine > Right_Bound_EndLine ? Left_Bound_EndLine : Right_Bound_EndLine;
            SetText_1("Left_Bound_StartLine = " + Left_Bound_StartLine);
            SetText_1("Left_Bound_EndLine = " + Left_Bound_EndLine);
            SetText_1("Right_Bound_StartLine = " + Right_Bound_StartLine);
            SetText_1("Right_Bound_EndLine = " + Right_Bound_EndLine);
            SetText_1("Scan_EndRow " + Scan_EndRow);
            SetText_1("****************************Exract Edge End*************************");
        }
        #endregion
        #region 十字处理函数及变量
        int Cross_cnt = -1;
        byte All_Cross_cnt = 10;
        byte Last_LeftMarginRow, Last_RightMarginRow = 1;
        byte Cro_Deal()
        {
            byte Flag = 0;
            SetText_1(" ");
            SetText_1("****************************Cro_Deal Start*************************");
            if ((Last_LeftMarginRow != 0 || Last_RightMarginRow != 0) && LeftMargin_1[0].row == 0 && RightMargin_1[0].row == 0)
            {
                Cross_cnt++;
                AllSpecRoadCnt++;
                //if (All_Cross_cnt == Cross_cnt++)
                //    Cross_cnt = 0;
            }
            setText用户自定义("Cross_cnt = "+ Cross_cnt);
            if (LeftLine_2.Agl_Line - RightLine_2.Agl_Line < 20)
            {
                SetText_1("Wrong CrossLine");
                RoadType = RoadTypeEnum.Common;
                SetText_1("****************************Cro_Deal End*************************");
                SetText_1(" ");
                return Flag;
            }
            if (LeftLine_1.Agl_Row != 0 && LeftLine_2.Agl_Row != 0)//左边前拐点和后拐点找到
            {
                Str_LineParaCale_Left(LeftLine_1.Agl_Row, LeftLine_1.Agl_Line, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);  //将前拐点和后拐点相连
                Str_LineSet_Left(LeftLine_1.Agl_Row, LeftLine_2.Agl_Row);
                SetText_1("Link L0: " + LeftLine_1.Agl_Row + " " + LeftLine_1.Agl_Line + " L1 :" + LeftLine_2.Agl_Row + " " + LeftLine_2.Agl_Line);
                Flag = 1;
            }
            else if (LeftLine_1.Agl_Row == 0 && LeftLine_2.Agl_Row != 0)  //左边前拐点没找到，后拐点找到，回拉后拐点
            {
                if (LeftLine_2.Agl_Row + 6 <= Scan_EndRow)
                {
                    Str_Regression_Left(LeftLine_2.Agl_Row, (byte)(LeftLine_2.Agl_Row + 6), 1);
                    Str_LineSet_Left(0, (byte)(LeftLine_2.Agl_Row));
                    SetText_1("Link L1: " + LeftLine_2.Agl_Row + " " + LeftLine_2.Agl_Line + " To " + (LeftLine_2.Agl_Row + 6) + " " + L_black[LeftLine_2.Agl_Row + 6]);
                }
                else if (LeftLine_2.Agl_Row + 3 <= Scan_EndRow)
                {
                    Str_Regression_Left(LeftLine_2.Agl_Row, (byte)(LeftLine_2.Agl_Row + 3), 1);
                    Str_LineSet_Left(0, (byte)(LeftLine_2.Agl_Row));
                    SetText_1("Link L1: " + LeftLine_2.Agl_Row + " " + LeftLine_2.Agl_Line + " To " + (LeftLine_2.Agl_Row + 3) + " " + L_black[LeftLine_2.Agl_Row + 3]);
                }
                Flag = 1;
            }
            else if (LeftLine_1.Agl_Row != 0 && LeftLine_2.Agl_Row == 0 && (RightLine_2.Agl_Row == 0 || RightLine_2.Agl_Row > LeftLine_1.Agl_Row))//只找到左边前拐点
            {
                if (LeftLine_1.Agl_Row >= 20)
                {
                    Str_Regression_Left((byte)(LeftLine_1.Agl_Row - 10), LeftLine_1.Agl_Row, 1);
                    Str_LineSet_Left(LeftLine_1.Agl_Row, 69);
                    SetText_1("Link L0 " + LeftLine_1.Agl_Row + " " + LeftLine_1.Agl_Line + " To " + (LeftLine_1.Agl_Row - 10) + " " + L_black[LeftLine_1.Agl_Row - 10]);
                }
                else
                {
                    Str_Regression_Left(1, LeftLine_1.Agl_Row, 1);
                    Str_LineSet_Left(LeftLine_1.Agl_Row, 69);
                    SetText_1("Link L0 " + LeftLine_1.Agl_Row + " " + LeftLine_1.Agl_Line + " To " + 1 + " " + L_black[1]);
                }
                Left_Bound_EndLine = 69;
            }

            if (RightLine_1.Agl_Row != 0 && RightLine_2.Agl_Row != 0)//右边前拐点和后拐点找到
            {
                Str_LineParaCale_Right(RightLine_1.Agl_Row, RightLine_1.Agl_Line, RightLine_2.Agl_Row, RightLine_2.Agl_Line);  //将前拐点和后拐点相连
                Str_LineSet_Right(RightLine_1.Agl_Row, RightLine_2.Agl_Row);
                SetText_1("Link R0: " + RightLine_1.Agl_Row + " " + RightLine_1.Agl_Line + " R1 :" + RightLine_2.Agl_Row + " " + RightLine_2.Agl_Line);
                Flag = 1;
            }
            else if (RightLine_1.Agl_Row == 0 && RightLine_2.Agl_Row != 0)  //右边前拐点没找到，找到后拐点，回拉后拐点
            {
                if (RightLine_2.Agl_Row + 6 <= Scan_EndRow)
                {
                    Str_Regression_Right(RightLine_2.Agl_Row, (byte)(RightLine_2.Agl_Row + 6),1);
                    Str_LineSet_Right(0, (byte)(RightLine_2.Agl_Row));
                    SetText_1("Link R1: " + RightLine_2.Agl_Row + " " + RightLine_2.Agl_Line + " To " + (RightLine_2.Agl_Row + 6) + " " + R_black[RightLine_2.Agl_Row + 6]);
                }
                else if (RightLine_2.Agl_Row + 3 <= Scan_EndRow)
                {
                    Str_Regression_Right(RightLine_2.Agl_Row, (byte)(RightLine_2.Agl_Row + 3),1);
                    Str_LineSet_Right(0, (byte)(RightLine_2.Agl_Row));
                    SetText_1("Link R1: " + RightLine_2.Agl_Row + " " + RightLine_2.Agl_Line + " To " + (RightLine_2.Agl_Row + 3) + " " + R_black[RightLine_2.Agl_Row + 3]);
                }
                Flag = 1;
            }
            else if (RightLine_1.Agl_Row != 0 && RightLine_2.Agl_Row == 0 && (LeftLine_2.Agl_Row == 0 || LeftLine_2.Agl_Row > RightLine_1.Agl_Row))//只找到右边前拐点
            {
                if (RightLine_1.Agl_Row >= 20)
                {
                    Str_Regression_Right((byte)(RightLine_1.Agl_Row - 10), RightLine_1.Agl_Row, 1);
                    Str_LineSet_Right(RightLine_1.Agl_Row, 69);
                    SetText_1("Link R0 " + RightLine_1.Agl_Row + " " + RightLine_2.Agl_Row + " To " + (RightLine_1.Agl_Row - 10) + " " + R_black[RightLine_1.Agl_Row - 10]);
                }
                else
                {
                    Str_Regression_Right(1, RightLine_1.Agl_Row, 1);
                    Str_LineSet_Right(RightLine_1.Agl_Row, 69);
                    SetText_1("Link R0 " + RightLine_1.Agl_Row + " " + RightLine_2.Agl_Row + " To " + 1 + " " + R_black[1]);
                }
                Right_Bound_EndLine = 69;
            }
            Last_LeftMarginRow = LeftMargin_1[0].row;
            Last_RightMarginRow = RightMargin_1[0].row;
            SetText_1("Str_Line_kb[0, 0] = " + Str_Line_kb[0, 0]);
            SetText_1("Str_Line_kb[1, 0] = "+ Str_Line_kb[1, 0]);
            SetText_1("****************************Cro_Deal End*************************");
            SetText_1(" ");
            return Flag;
        }
        #endregion
        #region 环岛处理函数及变量
        byte[] Circle_Rad = new byte[10] // 环岛大小 预设值，根据赛道实际情况调整
            {
                50,50,50,50,50,50,50,50,50,50
            };
        int Circle_cnt = -1; // 下一个要通过的环岛大小
        byte All_Circle_cnt = 10; // 总环岛大小
        void Circle_Pam_Get() // 对不同的环岛大小，设置不同的控制参数
        {
            if (RoadType == RoadTypeEnum.Circle_L)
            {
                switch (Circle_L_rad[Circle_L_cnt])
                {
                    case 50:
                        break;
                    case 60:
                        break;
                    case 70:
                        break;
                    case 80:
                        break;
                    case 90:
                        break;
                    case 100:
                        break;
                }
            }
            else if(RoadType == RoadTypeEnum.Circle_R)
            {
                switch (Circle_R_rad[Circle_R_cnt])
                {
                    case 50:
                        break;
                    case 60:
                        break;
                    case 70:
                        break;
                    case 80:
                        break;
                    case 90:
                        break;
                    case 100:
                        break;
                }
            }
        }
        void Circle_Left_Line_Cale(float line, float row) // 当左环岛不具备进行直线拟合的条件时，将右环岛的直线拟合结果作为参考，获得与之对称的拟合直线
        {
            Str_Line_kb[0, 0] = -Str_Line_kb[1, 0];
            //Str_Line_kb[0, 1] = (float)LeftLine_2.Agl_Line - Str_Line_kb[0, 0] * (float)LeftLine_2.Agl_Row;
            Str_Line_kb[0, 1] = line - Str_Line_kb[0, 0] * row;
        }
        void Circle_Left_Line_Clear()
        {
            int i, StartRow, EndRow;
            StartRow = LeftLine_2.Agl_Row > LeftMargin_2[0].row ? LeftMargin_2[0].row : LeftLine_2.Agl_Row;
            EndRow = LeftLine_2.Agl_Row <= LeftMargin_2[0].row ? LeftMargin_2[0].row : LeftLine_2.Agl_Row;
            for (i = StartRow - 1; i < EndRow; i++)
            {
                L_black[i] = 185;
            }
        }
        void Circle_Right_Line_Cale(float line, float row) // 当右环岛不具备进行直线拟合的条件时，将左环岛的直线拟合结果作为参考，获得与之对称的拟合直线
        {
            Str_Line_kb[1, 0] = -Str_Line_kb[0, 0];
            //Str_Line_kb[0, 1] = (float)RightLine_2.Agl_Line - Str_Line_kb[0, 0] * (float)RightLine_2.Agl_Row;
            Str_Line_kb[1, 1] = line - Str_Line_kb[1, 0] * row;
        }
        void Circle_Right_Line_Clear()
        {
            int i, StartRow, EndRow;
            StartRow = RightLine_2.Agl_Row > RightMargin_2[0].row ? RightMargin_2[0].row : RightLine_2.Agl_Row;
            EndRow = RightLine_2.Agl_Row <= RightMargin_2[0].row ? RightMargin_2[0].row : RightLine_2.Agl_Row;
            for (i = StartRow - 1; i < EndRow; i++)
            {
                R_black[i] = 0;
            }
        }
        byte Circle_Deal() // 环岛的赛道边线处理，使用状态机
        {
            SetText_1(" ");
            SetText_1("****************************Circle_Deal Start*************************");
            if (RoadType == RoadTypeEnum.Circle_L)
            {
                switch (Circle_State)
                {
                    case Circle.Circle_0:
                        if (LeftLine_2.Agl_Row != 0)
                        {
                            Str_LineParaCale_Left(LeftLine_1.Agl_Row, LeftLine_1.Agl_Line, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                            Str_LineSet_Left(LeftLine_1.Agl_Row, LeftLine_2.Agl_Row);
                        }
                        else
                        {
                            Str_Regression_Right((byte)1, 10, 1);
                            Circle_Left_Line_Cale((float)LeftLine_1.Agl_Line, (float)LeftLine_1.Agl_Row);
                            //Str_LineParaCale_Left(LeftLine_1.Agl_Row, LeftLine_1.Agl_Line, LeftMargin_2[0].row, LeftMargin_2[0].line);

                            Str_LineSet_Left(LeftLine_1.Agl_Row, 69);
                            Left_Bound_EndLine = 69;
                        }
                        break;
                    case Circle.Circle_1:
                        Str_Regression_Right((byte)(LeftLine_1.Agl_Row - 12), (byte)(LeftLine_1.Agl_Row - 6), 1);
                        Circle_Left_Line_Cale((float)LeftLine_1.Agl_Line, (float)LeftLine_1.Agl_Row);
                        Str_LineSet_Left(1, (byte)(LeftLine_1.Agl_Row));
                        break;
                    case Circle.Circle_2:
                        Str_LineParaCale_Right(10, R_black[10], 20, R_black[20]);
                        Circle_Left_Line_Cale((float)LeftLine_1.Agl_Line, (float)LeftLine_1.Agl_Row);
                        Str_LineSet_Left(1, (byte)(LeftLine_1.Agl_Row));

                        if(LeftLine_2.Agl_Row!=0)
                        {
                            /*if (LeftLine_1.Agl_Row < 5)
                            {
                                Str_LineParaCale_Right((byte)(LeftLine_1.Agl_Row-5), R_black[LeftLine_1.Agl_Row-5], LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                                Str_LineSet_Right((byte)(LeftLine_1.Agl_Row - 5), LeftLine_2.Agl_Row);
                            }
                            else
                            {*/
                            if (RightMargin_1[0].row <= 10&& RightMargin_1[0].row!=0)
                            {
                                Str_LineParaCale_Right(LeftLine_1.Agl_Row, R_black[LeftLine_1.Agl_Row], LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                                Str_LineSet_Right(LeftLine_1.Agl_Row, LeftLine_2.Agl_Row);
                            }
                            else
                            {
                                Str_LineParaCale_Right(1, 10, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                                Str_LineSet_Right(1, LeftLine_2.Agl_Row);
                            }
                            //}
                            Scan_EndRow = LeftLine_2.Agl_Row;
                        }
                        break;
                    case Circle.Circle_3:
                        if (LeftLine_2.Agl_Row != 0)
                        {
                            Str_LineParaCale_Right(1, 10, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                            Str_LineSet_Right(1, LeftLine_2.Agl_Row);
                            //Scan_EndRow = RightMargin_2[RightLine_2.Rec_Point].row > LeftLine_2.Agl_Row ? RightMargin_2[RightLine_2.Rec_Point].row : LeftLine_2.Agl_Row;
                        }
                        else
                        {
                            Str_LineParaCale_Right(1, 10, LeftMargin_2[LeftLine_2.Rec_Point].row, LeftMargin_2[LeftLine_2.Rec_Point].line);
                            Str_LineSet_Right(1, LeftMargin_2[LeftLine_2.Rec_Point].row);
                            //Scan_EndRow = LeftMargin_2[LeftLine_2.Rec_Point].row;
                        }
                        //SetText_1("Scan_EndRow = "+ Scan_EndRow);
                        break;
                    case Circle.Circle_4:
                        if (Scan_EndRow == 0 && NewSeedFind_2(Scan_LineType.RightType, Scan_Direction.Vertical, 105, 10, 69) == 1)
                        {
                            Str_LineParaCale_Right(RightMargin_2[RightLine_2.Rec_Point].row, RightMargin_2[RightLine_2.Rec_Point].line, 1, 184);
                            Str_LineSet_Right(1, RightMargin_2[RightLine_2.Rec_Point].row);
                        }
                        else if ((RightLine_1.Agl_Row != 0 && RightLine_1.Agl_Row < RightLine_2.EndLine) || (RightLine_1.PointCnt < 40))
                        {
                            Str_LineParaCale_Right(RightMargin_2[RightLine_2.Rec_Point].row, RightMargin_2[RightLine_2.Rec_Point].line, RightLine_1.Agl_Row, RightLine_1.Agl_Line);
                            Str_LineSet_Right(RightLine_1.Agl_Row, RightMargin_2[RightLine_2.Rec_Point].row);
                            Scan_EndRow = Left_Bound_EndLine = Right_Bound_EndLine = RightMargin_2[RightLine_2.Rec_Point].row;
                            //SetText_1("RightMargin_2["+RightLine_2.Rec_Point + "].row =" + RightMargin_2[RightLine_2.Rec_Point].row);
                            SetText_1("Right_Bound_EndLine = " + Right_Bound_EndLine);
                        }
                        Left_Bound_EndLine = Scan_EndRow;
                        break;
                    case Circle.Circle_5:
                        Str_LineParaCale_Right(RightMargin_1[RightLine_1.Rec_Point].row, RightMargin_1[RightLine_1.Rec_Point].line, 1, 1);
                        Str_LineSet_Right(1, RightMargin_1[RightLine_1.Rec_Point].row);
                        Scan_EndRow = RightMargin_1[RightLine_1.Rec_Point].row;
                        break;
                    case Circle.Circle_6:
                        if (RightMargin_1[0].row > 1)
                        {
                            if (RightLine_1.Agl_Row == 0)
                            {
                                Str_LineParaCale_Right(RightMargin_1[RightLine_1.Rec_Point].row, RightMargin_1[RightLine_1.Rec_Point].line, 1, 1);
                                Str_LineSet_Right(1, RightMargin_1[RightLine_1.Rec_Point].row);
                                Scan_EndRow = Right_Bound_EndLine = RightMargin_1[RightLine_1.Rec_Point].row;
                            }
                            else
                            {
                                Str_LineParaCale_Right(RightLine_1.Agl_Row, RightLine_1.Agl_Line, 1, 1);
                                Str_LineSet_Right(1, RightLine_1.Agl_Row);
                                Scan_EndRow = Right_Bound_EndLine = RightMargin_1[RightLine_1.Rec_Point].row;
                            }
                        }
                        else
                            Scan_EndRow = RightMargin_1[RightLine_1.Rec_Point].row;
                        if (LeftLine_2.Agl_Row != 0)
                        {
                            Str_Regression_Left(LeftLine_2.Agl_Row, (byte)(LeftLine_2.Agl_Row + 6), 1);
                            SetText_1("Str_Line_kb[0, 0] = " + Str_Line_kb[0, 0]);
                            if (Str_Line_kb[0, 0] >= 0 || Str_Line_kb[0, 0] <= -2.2)
                            {
                                SetText_1("!Wrong reg k,Recale the line");
                                if (LeftLine_1.Agl_Row != 0 && LeftMargin_1[0].row <= 10)
                                {
                                    Str_LineParaCale_Left(LeftMargin_1[0].row, LeftMargin_1[0].line, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                                }
                                else
                                {
                                    Str_LineParaCale_Left(1, 184, LeftLine_2.Agl_Row, LeftLine_2.Agl_Line);
                                }
                            }
                            Str_LineSet_Left(1, (byte)(LeftLine_2.Agl_Row));
                        }
                        /*else if (LeftLine_2.Error >= 2)
                        {
                            Str_LineParaCale_Left(50, (byte)(R_black[50]+50), 1, 184);
                            Str_LineSet_Left(1, 50);
                        }*/
                        break;
                    case Circle.Circle_7:
                        Str_Regression_Left(LeftLine_2.Agl_Row, (byte)(LeftLine_2.Agl_Row + 6), 1);
                        Str_LineSet_Left(1, (byte)(LeftLine_2.Agl_Row + 6));
                        break;
                    default: break;
                }
            }
            else
            {
                switch (Circle_State)
                {
                    case Circle.Circle_0:
                        if (RightLine_2.Agl_Row != 0)
                        {
                            Str_LineParaCale_Right(RightLine_1.Agl_Row, RightLine_1.Agl_Line, RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                            Str_LineSet_Right(RightLine_1.Agl_Row, RightLine_2.Agl_Row);
                        }
                        else
                        {
                            Str_Regression_Left((byte)1, 10, 1);
                            Circle_Right_Line_Cale((float)RightLine_1.Agl_Line, (float)RightLine_1.Agl_Row);
                            //Str_LineParaCale_Right(RightLine_1.Agl_Row, RightLine_1.Agl_Line, RightMargin_2[0].row, RightMargin_2[0].line);

                            Str_LineSet_Right(RightLine_1.Agl_Row, 69);
                            Right_Bound_EndLine = 69;
                        }
                        break;
                    case Circle.Circle_1:
                        Str_Regression_Left((byte)(RightLine_1.Agl_Row - 12), (byte)(RightLine_1.Agl_Row - 6), 1);
                        Circle_Right_Line_Cale((float)RightLine_1.Agl_Line, (float)RightLine_1.Agl_Row);
                        Str_LineSet_Right(1, (byte)(RightLine_1.Agl_Row));
                        break;
                    case Circle.Circle_2:
                        Str_LineParaCale_Left(10, L_black[10], 20, L_black[20]);
                        Circle_Right_Line_Cale((float)RightLine_1.Agl_Line, (float)RightLine_1.Agl_Row);
                        Str_LineSet_Right(1, (byte)(RightLine_1.Agl_Row));

                        if (RightLine_2.Agl_Row != 0)
                        {
                            /*if (RightLine_1.Agl_Row > 5)
                            {
                                Str_LineParaCale_Left((byte)(RightLine_1.Agl_Row-5), L_black[RightLine_1.Agl_Row - 5], RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                                Str_LineSet_Left((byte)(RightLine_1.Agl_Row - 5), RightLine_2.Agl_Row);
                            }
                            else
                            {*/
                            //if (LeftMargin_1[0].row <= 10&&LeftMargin_1[0].row!=0)
                            //{
                            Str_LineParaCale_Left(RightLine_1.Agl_Row, L_black[RightLine_1.Agl_Row], RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                            Str_LineSet_Left(RightLine_1.Agl_Row, RightLine_2.Agl_Row);
                            //}
                            //else
                            //{
                            //Str_LineParaCale_Left(1, 175, RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                            //Str_LineSet_Left(1, RightLine_2.Agl_Row);
                            //}
                            //}
                            Scan_EndRow = RightLine_2.Agl_Row;
                        }
                        break;
                    case Circle.Circle_3:
                        if (RightLine_2.Agl_Row != 0)
                        {
                            Str_LineParaCale_Left(1, 175, RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                            Str_LineSet_Left(1, RightLine_2.Agl_Row);
                            Scan_EndRow = LeftMargin_2[LeftLine_2.Rec_Point].row > RightLine_2.Agl_Row ? LeftMargin_2[LeftLine_2.Rec_Point].row : RightLine_2.Agl_Row;
                        }
                        else
                        {
                            Str_LineParaCale_Left(1, 175, RightMargin_2[RightLine_2.PointCnt].row, RightMargin_2[RightLine_2.PointCnt].line);
                            Str_LineSet_Left(1, RightMargin_2[RightLine_2.PointCnt].row);
                            Scan_EndRow = RightMargin_2[RightLine_2.PointCnt].row;
                        }
                        //SetText_1("Scan_EndRow = " + Scan_EndRow);
                        break;
                    case Circle.Circle_4:
                        if (Scan_EndRow == 0 && NewSeedFind_2(Scan_LineType.LeftType, Scan_Direction.Vertical, 80, 10, 69) == 1)
                        {
                            Str_LineParaCale_Left(LeftMargin_2[LeftLine_2.Rec_Point].row, LeftMargin_2[LeftLine_2.Rec_Point].line, 1, 1);
                            Str_LineSet_Left(1, LeftMargin_2[LeftLine_2.Rec_Point].row);
                        }
                        else if ((LeftLine_1.Agl_Row != 0 && LeftLine_1.Agl_Row < LeftLine_2.EndLine) || (LeftLine_1.PointCnt < 40))
                        {
                            SetText_1("LeftMargin_2[LeftLine_2.Rec_Point].row = " + LeftMargin_2[LeftLine_2.Rec_Point].row + " LeftMargin_2[LeftLine_2.Rec_Point].line = " + LeftMargin_2[LeftLine_2.Rec_Point].line);
                            SetText_1("LeftLine_2.Rec_Point = " + LeftLine_2.Rec_Point);
                            Str_LineParaCale_Left(LeftMargin_2[LeftLine_2.Rec_Point].row, LeftMargin_2[LeftLine_2.Rec_Point].line, LeftLine_1.Agl_Row, LeftLine_1.Agl_Line);
                            Str_LineSet_Left(LeftLine_1.Agl_Row, LeftMargin_2[LeftLine_2.Rec_Point].row);
                            Scan_EndRow = Right_Bound_EndLine = Left_Bound_EndLine = LeftMargin_2[LeftLine_2.Rec_Point].row;
                        }
                        Right_Bound_EndLine = Scan_EndRow;
                        break;
                    case Circle.Circle_5:
                        //SetText_1("LeftMargin_1[LeftLine_1.Rec_Point].row = "+ LeftMargin_1[LeftLine_1.Rec_Point].row + " LeftMargin_1[LeftLine_1.Rec_Point].line = "+ LeftMargin_1[LeftLine_1.Rec_Point].line);
                        //SetText_1("LeftLine_1.Rec_Point = "+ LeftLine_1.Rec_Point);
                        Str_LineParaCale_Left(LeftMargin_1[LeftLine_1.Rec_Point].row, LeftMargin_1[LeftLine_1.Rec_Point].line, 1, 184);
                        Str_LineSet_Left(1, LeftMargin_1[LeftLine_1.Rec_Point].row);
                        Scan_EndRow = LeftMargin_1[LeftLine_1.Rec_Point].row;
                        break;
                    case Circle.Circle_6:
                        SetText_1("LeftMargin_1[0].row = "+ LeftMargin_1[0].row);
                        if (LeftMargin_1[0].row > 1)
                        {
                            //SetText_1("LeftMargin_2[LeftLine_2.Rec_Point].row = " + LeftMargin_2[LeftLine_2.Rec_Point].row + " LeftMargin_2[LeftLine_2.Rec_Point].line = " + LeftMargin_2[LeftLine_2.Rec_Point].line);
                            if (LeftLine_1.Agl_Row == 0)
                            {
                                Str_LineParaCale_Left(LeftMargin_1[LeftLine_1.Rec_Point].row, LeftMargin_1[LeftLine_1.Rec_Point].line, 1, 184);
                                Str_LineSet_Left(1, LeftMargin_1[LeftLine_1.Rec_Point].row);
                                Scan_EndRow = Left_Bound_EndLine = LeftMargin_1[LeftLine_1.Rec_Point].row;
                            }
                            else
                            {
                                Str_LineParaCale_Left(LeftLine_1.Agl_Row, LeftLine_1.Agl_Line, 1, 184);
                                Str_LineSet_Left(1, LeftLine_1.Agl_Row);
                                Scan_EndRow = Left_Bound_EndLine = LeftMargin_1[LeftLine_1.Agl_PointNum].row;
                            }
                        }
                        else
                            Scan_EndRow = LeftMargin_1[LeftLine_1.Rec_Point].row;
                        if (RightLine_2.Agl_Row != 0)
                        {
                            Str_Regression_Right(RightLine_2.Agl_Row, (byte)(RightLine_2.Agl_Row + 6), 1);
                            SetText_1("Str_Line_kb[1, 0] = " + Str_Line_kb[1, 0]);
                            if (Str_Line_kb[1, 0] <= 0 || Str_Line_kb[1, 0] >= 2.2)
                            {
                                SetText_1("!Wrong reg k,Recale the line");
                                if (RightLine_1.Agl_Row != 0 && RightMargin_1[0].row <= 10)
                                {
                                    Str_LineParaCale_Right(RightMargin_1[0].row, RightMargin_1[0].line, RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                                }
                                else
                                {
                                    Str_LineParaCale_Right(1, 1, RightLine_2.Agl_Row, RightLine_2.Agl_Line);
                                }
                            }
                            Str_LineSet_Right(1, (byte)(RightLine_2.Agl_Row));
                            SetText_1("Str_Line_kb[1, 0] = " + Str_Line_kb[1, 0]);
                        }
                        /*else if (RightLine_2.Error == 1)
                        {
                            SetText_1("1");
                            Str_LineParaCale_Right(40, (byte)(L_black[40]-40), 1, 1);
                            Str_LineSet_Right(1, 40);
                            SetText_1("Error Line 2 ! ");
                        }*/
                        break;
                    case Circle.Circle_7:
                        Str_Regression_Right(RightLine_2.Agl_Row, (byte)(RightLine_2.Agl_Row + 6), 1);
                        Str_LineSet_Right(1, (byte)(RightLine_2.Agl_Row + 6));
                        break;
                    default: break;
                }
            }
            SetText_1("****************************Circle_Deal End*************************");
            SetText_1(" ");
            return 1;
        }
        #endregion
        #region 车库函数及变量
        byte GarageRow, GarageLine;
        byte Garage_Deal() // 入车库，赛道边线处理
        {
            SetText_1(" ");
            SetText_1("****************************Garage Deal Start*************************");
            if (RoadType == RoadTypeEnum.Garage_L)
            {
                switch (Garage_State)
                {
                    case Garage.Garage_0:
                        if (LeftLine_2.Agl_Row != 0)
                        {
                            if (LeftLine_1.Agl_Row >= 20)
                            {
                                Str_LineParaCale_Right(LeftLine_2.Agl_Row, LeftLine_2.Agl_Line, LeftLine_1.Agl_Row, R_black[LeftLine_1.Agl_Row]);
                                Str_LineSet_Right(LeftLine_1.Agl_Row, LeftLine_2.Agl_Row);
                            }
                            else
                            {
                                Str_LineParaCale_Right(LeftLine_2.Agl_Row, LeftLine_2.Agl_Line, 1, R_black[1]);
                                Str_LineSet_Right(1, LeftLine_2.Agl_Row);
                                for (OlRow = 0; OlRow <= LeftLine_2.Agl_Row; OlRow++)
                                {
                                    L_black[OlRow] = 184;
                                }
                            }
                            Scan_EndRow = LeftLine_2.Agl_Row;
                        }
                        break;
                    case Garage.Garage_1:
                        if (LeftMargin_2[0].row != 0)
                        {
                            if (LeftLine_2.Agl_Row != 0 && LeftLine_2.Agl_Row < 30)
                            {
                                Str_Regression_Right(RightMargin_2[0].row, (byte)(RightMargin_2[0].row + 5), 1);
                                Str_LineSet_Right(0, RightMargin_2[0].row);
                            }
                            else
                            {
                                Str_LineParaCale_Right(LeftMargin_2[0].row, LeftMargin_2[0].line, 1, 1);
                                Str_LineSet_Right(0, LeftMargin_2[0].row);
                                Scan_EndRow = LeftMargin_2[0].row;
                            }
                        }
                        for (OlRow = 0; OlRow <= LeftMargin_2[0].row; OlRow++)
                        {
                            L_black[OlRow] = 184;
                        }
                        Left_Bound_EndLine = 1;
                        Scan_EndRow = Right_Bound_EndLine = LeftMargin_2[0].row;
                        break;
                    case Garage.Garage_2:
                        break;
                    case Garage.Garage_3:
                        break;
                }
            }
            else
            {
                switch (Garage_State)
                {
                    case Garage.Garage_0:
                        if (RightLine_2.Agl_Row != 0)
                        {
                            if (RightLine_1.Agl_Row >= 20)
                            {
                                Str_LineParaCale_Left(RightLine_2.Agl_Row, RightLine_2.Agl_Line, RightLine_1.Agl_Row, L_black[RightLine_1.Agl_Row]);
                                Str_LineSet_Left(RightLine_1.Agl_Row, RightLine_2.Agl_Row);
                            }
                            else
                            {
                                Str_LineParaCale_Left(RightLine_2.Agl_Row, RightLine_2.Agl_Line, 1, L_black[1]);
                                Str_LineSet_Left(1, RightLine_2.Agl_Row);
                                for (OlRow = 0; OlRow <= RightLine_2.Agl_Row; OlRow++)
                                {
                                    R_black[OlRow] = 1;
                                }
                            }
                            Scan_EndRow = RightLine_2.Agl_Row;
                        }
                        break;
                    case Garage.Garage_1:
                        if (RightMargin_2[0].row != 0)
                        {
                            if (RightLine_2.Agl_Row != 0 && RightLine_2.Agl_Row < 30)
                            {
                                Str_Regression_Left(LeftMargin_2[0].row, (byte)(LeftMargin_2[0].row + 5), 184);
                                Str_LineSet_Left(0, LeftMargin_2[0].row);
                            }
                            else
                            {
                                Str_LineParaCale_Left(RightMargin_2[0].row, RightMargin_2[0].line, 0, 184);
                                Str_LineSet_Left(0, RightMargin_2[0].row);
                                Scan_EndRow = RightMargin_2[0].row;
                            }
                        }
                        for (OlRow = 0; OlRow <= RightMargin_2[0].row; OlRow++)
                        {
                            R_black[OlRow] = 1;
                        }
                        Right_Bound_EndLine = 1;
                        break;
                    case Garage.Garage_2:
                        break;
                    case Garage.Garage_3:
                        break;
                }
            }
            SetText_1(" ");
            SetText_1("****************************Garage Deal End*************************");
            return 1;
        }
        byte Out_Garage_Deal() // 出车库，赛道边线处理
        {
            SetText_1(" ");
            SetText_1("****************************Out Garage Deal Start*************************");
            if (RoadType == RoadTypeEnum.Out_Garage_L)
            {
                switch (Out_Garage_State)
                {
                    case Out_Garage.Out_Garage_0:
                        for (OlRow = 0; OlRow < 30; OlRow++)
                        {
                            R_black[OlRow] = L_black[OlRow] = 93;
                        }
                        Scan_EndRow = Right_Bound_EndLine = Left_Bound_EndLine = 30;
                        break;
                    case Out_Garage.Out_Garage_1:
                        if (RightMargin_2[0].row != 0)
                        {
                            Str_LineParaCale_Right(1, 5, RightMargin_2[0].row, RightMargin_2[0].line);
                            Str_LineSet_Right(1, RightMargin_2[0].row);
                            Scan_EndRow = Right_Bound_EndLine = RightMargin_2[0].row;
                        }
                        else
                        {
                            Str_LineParaCale_Right(1, 5, 69, 180);
                            Str_LineSet_Right(1, 69);
                            Scan_EndRow = Right_Bound_EndLine = 69;
                        }
                        break;
                    case Out_Garage.Out_Garage_2:
                        if(RightLine_1.Agl_Row!=0)
                        {
                            Str_LineParaCale_Right(1, 5, RightLine_1.Agl_Row, RightLine_1.Agl_Line);
                            Str_LineSet_Right(1, RightLine_1.Agl_Row);
                            Scan_EndRow = Right_Bound_EndLine = RightLine_1.Agl_Row;
                        }
                        else if (RightMargin_2[0].row != 0)
                        {
                            Str_LineParaCale_Right(1, 5, RightMargin_2[0].row, RightMargin_2[0].line);
                            Str_LineSet_Right(1, RightMargin_2[0].row);
                            Scan_EndRow = Right_Bound_EndLine = RightMargin_2[0].row;
                        }
                        else
                        {
                            Str_LineParaCale_Right(1, 5, 69, 180);
                            Str_LineSet_Right(1, 69);
                            Scan_EndRow = Right_Bound_EndLine = 69;
                        }
                        break;
                    case Out_Garage.Out_Garage_3:
                        break;
                }
            }
            else
            {
                switch (Out_Garage_State)
                {
                    case Out_Garage.Out_Garage_0:
                        for (OlRow = 0; OlRow < 30; OlRow++)
                        {
                            R_black[OlRow] = L_black[OlRow] = 93;
                        }
                        Scan_EndRow = Right_Bound_EndLine = Left_Bound_EndLine = 30;
                        break;
                    case Out_Garage.Out_Garage_1:
                        if (LeftMargin_2[0].row != 0)
                        {
                            Str_LineParaCale_Left(1, 180, LeftMargin_2[0].row, LeftMargin_2[0].line);
                            Str_LineSet_Left(1, LeftMargin_2[0].row);
                            Scan_EndRow = Left_Bound_EndLine = LeftMargin_2[0].row;
                        }
                        else
                        {
                            Str_LineParaCale_Left(1, 180, 69, 5);
                            Str_LineSet_Left(1, 69);
                            Scan_EndRow = Left_Bound_EndLine = 69;
                        }
                        break;
                    case Out_Garage.Out_Garage_2:
                        if(LeftLine_1.Agl_Row!=0)
                        {
                            Str_LineParaCale_Left(1, 180, LeftLine_1.Agl_Row, LeftLine_1.Agl_Line);
                            Str_LineSet_Left(1, LeftLine_1.Agl_Row);
                            Scan_EndRow = Left_Bound_EndLine = LeftLine_1.Agl_Row;
                        }
                        else if (LeftMargin_2[0].row != 0)
                        {
                            Str_LineParaCale_Left(1, 180, LeftMargin_2[0].row, LeftMargin_2[0].line);
                            Str_LineSet_Left(1, LeftMargin_2[0].row);
                            Scan_EndRow = Left_Bound_EndLine = LeftMargin_2[0].row;
                        }
                        else
                        {
                            Str_LineParaCale_Left(1, 180, 69, 5);
                            Str_LineSet_Left(1, 69);
                            Scan_EndRow = Left_Bound_EndLine = 69;
                        }
                        break;
                    case Out_Garage.Out_Garage_3:
                        break;
                }
            }
            SetText_1("****************************Out Garage Deal End*************************");
            return 1;
        }
        #endregion
        #region 坡道函数及变量
        float Gyro_Ave;
        int Slope_Fps = 0;
        int Slope_Count = -1;
        int Slope_TotalCnt = 5;
        int Slope_Cstrain_Fps = 0;
        int Slope_All_Cstrain_Fps = 60;
        void Slope_Judge() // 坡道判断
        {
            if (RoadType != RoadTypeEnum.Slope
             && RoadType != RoadTypeEnum.Circle_L && RoadType != RoadTypeEnum.Circle_R
             && RoadType != RoadTypeEnum.Garage_L && RoadType != RoadTypeEnum.Garage_R
             && RoadType != RoadTypeEnum.Out_Garage_L && RoadType != RoadTypeEnum.Out_Garage_R
             && RoadType != RoadTypeEnum.Meeting)
            {
                SetText_1(" ");
                SetText_1("****************************Slope Judge Start*************************");
                byte count = 0;
                if (RoadType == RoadTypeEnum.Common || RoadType == RoadTypeEnum.Straight || RoadType == RoadTypeEnum.Turn_L || RoadType == RoadTypeEnum.Turn_R)
                {
                    if (Slope_Fps > 0)
                    {
                        Slope_Fps--;
                    }
                    else
                    {
                        if (RoadType == RoadTypeEnum.Straight)
                        {
                            for (OlRow = 50; OlRow < 65; OlRow++)
                            {
                                SetText_2("Error[" + OlRow + "] = " + (L_black[OlRow] - R_black[OlRow]));
                                if ((L_black[OlRow] - R_black[OlRow]) > 50 && L_black[OlRow] <= 184 && R_black[OlRow] >= 1)
                                {
                                    count++;
                                }
                            }
                            SetText_1("Slope Judge count = " + count);
                            if (count >= 12)
                            {
                                Str_Regression_Left(50, (byte)(Left_Bound_EndLine > 65 ? 65 : Left_Bound_EndLine), 0);
                                Str_Regression_Right(50, (byte)(Right_Bound_EndLine > 65 ? 65 : Right_Bound_EndLine), 0);
                                
                                setText用户自定义("Str_Line_kb[0][0] = " + Str_Line_kb[0, 0]);
                                setText用户自定义("Str_Line_kb[1][0] = " + Str_Line_kb[1, 0]);
                                if (
                                    my_fabs(Str_Line_kb[0, 0] + Str_Line_kb[1, 0]) < 0.8
                                  //LeftLine_1.Agl_Row == 0 && RightLine_1.Agl_Row == 0
                                  && Str_Quant(50, (byte)(Left_Bound_EndLine > 65 ? 65 : Left_Bound_EndLine), 0) < 1.5
                                  && Str_Quant(50, (byte)(Right_Bound_EndLine > 65 ? 65 : Right_Bound_EndLine), 1) < 1.5)
                                {
                                    setText用户自定义("Start Slope");
                                    RoadType = RoadTypeEnum.Slope;
                                    Slope_State = Slope_st.Slope_0;
                                    Slope_Fps = 0;
                                    Slope_Cstrain_Fps = Slope_All_Cstrain_Fps;

                                    Slope_Count++;
                                    AllSpecRoadCnt++;
                                    //if (Slope_TotalCnt == Slope_Count++)
                                    //    Slope_Count = 0;
                                }
                            }
                        }
                        else
                        {
                            for (OlRow = 40; OlRow < 55; OlRow++)
                            {
                                SetText_2("Error[" + OlRow + "] = " + (L_black[OlRow] - R_black[OlRow]));
                                if ((L_black[OlRow] - R_black[OlRow]) > 80 && L_black[OlRow] <= 184 && R_black[OlRow] >= 1)
                                {
                                    count++;
                                }
                            }
                            SetText_1("Slope Judge count = " + count);
                            if (count >= 10)
                            {
                                Str_Regression_Left(40, (byte)(Left_Bound_EndLine > 55 ? 55 : Left_Bound_EndLine), 0);
                                Str_Regression_Right(40, (byte)(Right_Bound_EndLine > 55 ? 55 : Right_Bound_EndLine), 0);
                                setText用户自定义("Str_Line_kb[0][0] = " + Str_Line_kb[0, 0]);
                                setText用户自定义("Str_Line_kb[1][0] = " + Str_Line_kb[1, 0]);
                                if (
                                    my_fabs(Str_Line_kb[0, 0] + Str_Line_kb[1, 0]) < 1
                                  //LeftLine_1.Agl_Row == 0 && RightLine_1.Agl_Row == 0
                                  && Str_Quant(40, (byte)(Left_Bound_EndLine > 55 ? 55 : Left_Bound_EndLine), 0) < 1.5
                                  && Str_Quant(40, (byte)(Right_Bound_EndLine > 55 ? 55 : Right_Bound_EndLine), 1) < 1.5)
                                {
                                    setText用户自定义("Start Slope");
                                    RoadType = RoadTypeEnum.Slope;
                                    Slope_State = Slope_st.Slope_0;
                                    Slope_Fps = 0;
                                    Slope_Cstrain_Fps = Slope_All_Cstrain_Fps;

                                    Slope_Count++;
                                    AllSpecRoadCnt++;
                                    //if (Slope_TotalCnt == Slope_Count++)
                                    //    Slope_Count = 0;
                                }
                            }
                        }

                    }
                }
                SetText_1("****************************Slope Judge End*************************");
            }
        }
        void Slope_Deal() // 坡道的赛道边线处理
        {
            SetText_1(" ");
            SetText_1("****************************Slope Deal Start*************************");
            Slope_Cstrain_Fps--;
            if (Slope_Cstrain_Fps == 0)
            {
                Slope_State = Slope_st.Slope_0;
                RoadType = RoadTypeEnum.Common;
                return;
            }
            byte count=0;
            if (Slope_State == Slope_st.Slope_0 && Scan_EndRow <= 65)
            {
                Slope_State = Slope_st.Slope_1;
            }
            else if(Slope_State == Slope_st.Slope_2 && Scan_EndRow >= 50)
            {
                Slope_State = Slope_st.Slope_3;
            }
            switch (Slope_State)
            {
                case Slope_st.Slope_0:
                    break;
                case Slope_st.Slope_1:
                    for (OlRow = 1; OlRow < 20; OlRow++)
                    {
                        SetText_2("Error[" + OlRow + "] = " + (L_black[OlRow] - R_black[OlRow]));
                        if(my_fabs(L_black[OlRow] - R_black[OlRow])<90)
                        {
                            count++;
                        }
                    }
                    setText用户自定义("Slope count = " + count);
                    setText用户自定义("Scan_EndRow = " + Scan_EndRow);
                    if (count>=8||Scan_EndRow<=20)
                    {
                       Slope_State = Slope_st.Slope_2;
                    }
                    break;
                case Slope_st.Slope_2:
                    for (OlRow = 1; OlRow < 11; OlRow++)
                    {
                        SetText_1("OlRow = "+ OlRow);
                        if ((L_black[OlRow] + R_black[OlRow]) / 2 < 65 || (L_black[OlRow] + R_black[OlRow]) / 2 > 121)
                        {
                            for (byte x = 1; x < 11; x++)
                            {
                                L_black[x] = R_black[x] = 93;
                            }
                            break;
                        }
                    }
                    Scan_EndRow = 10;
                    break;
                case Slope_st.Slope_3:
                    for (OlRow = 1; OlRow < 15; OlRow++)
                    {
                        SetText_2("Error[" + OlRow + "] = " + (L_black[OlRow] - R_black[OlRow]));
                        if ((L_black[OlRow] - R_black[OlRow]) >= 160)
                        {
                            count++;
                        }
                    }
                    setText用户自定义("Slope count = " + count);
                    if (count >= 8)
                    {
                        Slope_State = Slope_st.Slope_4;
                        Slope_Fps = 0;
                    }
                    break;
                case Slope_st.Slope_4:
                    Slope_Fps++;
                    for (OlRow = 1; OlRow < 15; OlRow++)
                    {
                        SetText_2("Error[" + OlRow + "] = " + (L_black[OlRow] - R_black[OlRow]));
                        if ((L_black[OlRow] - R_black[OlRow]) >= 140&& (L_black[OlRow] - R_black[OlRow]) <= 170)
                        {
                            count++;
                        }
                    }
                    setText用户自定义("Slope count = " + count);
                    if (count >= 8 || Slope_Fps > 8)
                    {
                        Slope_Fps = 20;
                        Slope_State = Slope_st.Slope_0;
                        RoadType = RoadTypeEnum.Common;
                        setText用户自定义("Quit Slope");
                    }
                    break;
            }
            setText用户自定义("Slope_Cstrain_Fps = " + Slope_Cstrain_Fps);
            setText用户自定义("Slope_Fps = " + Slope_Fps);
            setText用户自定义("Slope_State = " + Slope_State);
            SetText_1("****************************Slope Deal End*************************");
        }
        #endregion
        #region  双车会车函数
        float[] Mid_StrLine_kb = new float[2];
        float MeetingMode_0_k = -1.02f;
        float MeetingMode_0_b = 74.6f;
        float MeetingMode_1_k = -1.65f;
        float MeetingMode_1_b = 126.6f;
        byte MeetingFlag = 0;
        byte CarTrailRow = 1;
        byte CarTrailLine = 93;
        byte MeetingMode = 0;
        //byte Meeting_Fps = 0;
        //byte Meeting_st3_Fps = 10;
        void Str_LineParaCale_Mid(byte x1, byte y1, byte x2, byte y2) // 会车时，中线用到的直线计算函数
        {
            SetText_1("Mid_StrLine_kb[0] = "+ Mid_StrLine_kb[0]+ " Mid_StrLine_kb[1] = "+ Mid_StrLine_kb[1]);
            Mid_StrLine_kb[0] = (float)(y2 - y1) / (float)(x2 - x1);
            Mid_StrLine_kb[1] = (float)y2 - Mid_StrLine_kb[0] * (float)x2;
        }
        byte Car_widthCale(byte Mode) // 计算另一辆车的宽度
        {
            float k ;
            float b ;
            int exp_width;
            float act_width;
            if(Mode==0)
            {
                k = MeetingMode_0_k;
                b = MeetingMode_0_b;
            }
            else 
            {
                k = MeetingMode_1_k;
                b = MeetingMode_1_b;
            }
            if (Car_RightLine.Agl_Row != 0 && Car_LeftLine.Agl_Row != 0)
            {
                exp_width = (byte)(k * ((Car_RightLine.Agl_Row + Car_LeftLine.Agl_Row) / 2) + b);
                act_width = (float)Math.Sqrt((Car_RightLine.Agl_Row - Car_LeftLine.Agl_Row) * (Car_RightLine.Agl_Row - Car_LeftLine.Agl_Row) + ((Car_RightLine.Agl_Line - Car_LeftLine.Agl_Line) * (Car_RightLine.Agl_Line - Car_LeftLine.Agl_Line)));
                SetText_1("Exp_Width = " + exp_width + " Actual_Width =  " + act_width);
                if (RoadType == RoadTypeEnum.Meeting)
                {
                    if (my_fabs(exp_width - act_width) <= 15)
                    {
                        SetText_1("Width Proves to be car");
                        return 1;
                    }
                }
                else
                {
                    if (my_fabs(exp_width - act_width) <= 10)
                    {
                        SetText_1("Width Proves to be car");
                        return 1;
                    }
                }
            }
            else if (Car_RightLine.Agl_Row == 0 && Car_LeftLine.Agl_Row != 0)
            {
                exp_width = (byte)(k * ((Car_RightMargin[Car_RightLine.PointCnt].row + Car_LeftLine.Agl_Row) / 2) + b);
                act_width = (float)Math.Sqrt((Car_RightMargin[Car_RightLine.PointCnt].row - Car_LeftLine.Agl_Row) * (Car_RightMargin[Car_RightLine.PointCnt].row - Car_LeftLine.Agl_Row) + ((Car_LeftLine.Agl_Line - Car_RightMargin[Car_RightLine.PointCnt].line) * (Car_LeftLine.Agl_Line - Car_RightMargin[Car_RightLine.PointCnt].line)));
                SetText_1("Exp_Width = " + exp_width + " Actual_Width =  " + act_width);
                if (my_fabs(exp_width - act_width) <= 15)
                {
                    SetText_1("Width Proves to be car");
                    return 1;
                }
            }
            else if (Car_RightLine.Agl_Row != 0 && Car_LeftLine.Agl_Row == 0)
            {
                exp_width = (byte)(k * ((Car_RightLine.Agl_Row + Car_LeftMargin[Car_LeftLine.PointCnt].row) / 2) + b);
                act_width = (float)Math.Sqrt((Car_RightLine.Agl_Row - Car_LeftMargin[Car_LeftLine.PointCnt].row) * (Car_RightLine.Agl_Row - Car_LeftMargin[Car_LeftLine.PointCnt].row) + ((Car_LeftMargin[Car_LeftLine.PointCnt].line - Car_RightLine.Agl_Line) * (Car_LeftMargin[Car_LeftLine.PointCnt].line - Car_RightLine.Agl_Line)));
                SetText_1("Exp_Width = " + exp_width + " Actual_Width =  " + act_width);
                if (my_fabs(exp_width - act_width) <= 15)
                {
                    SetText_1("Width Proves to be car");
                    return 1;
                }
            }
            SetText_1("Width Wrong!");
            return 0;
        }
        byte Meeting_BiasSeed_Find(byte LowerData, byte UpperData, byte i, float k)  //0为以右边线为基准，1为以左边线。为车线扫描寻找种子
        {
            int x;
            SetText_1(" ");
            SetText_1("Mod = " + i);
            if (i == 0)
            {
                Str_Regression_Left(1, LeftLine_1.EndLine,0);
                Mid_StrLine_kb[0] = Str_Line_kb[0, 0] * k;
                Mid_StrLine_kb[1] = (L_black[1] + R_black[1]) / 2 - Mid_StrLine_kb[0] * 1;
            }
            else if (i == 1)
            {
                Str_Regression_Right(1, RightLine_1.EndLine,0);
                Mid_StrLine_kb[0] = Str_Line_kb[1, 0] * k;
                Mid_StrLine_kb[1] = (L_black[1] + R_black[1]) / 2 - Mid_StrLine_kb[0] * 1;
            }
            else if (i == 2)
            {
                Str_LineParaCale_Mid(1, 93, CarTrailRow, CarTrailLine);
            }
            else if(i==3)
            {
                Str_Regression_Left(1, 40, 0);
                Str_Regression_Right(1, 40, 0);
                Mid_StrLine_kb[0] = (Str_Line_kb[0, 0] + Str_Line_kb[1, 0]) * k;
                Mid_StrLine_kb[1] = (L_black[1] + R_black[1]) / 2 - Mid_StrLine_kb[0] * 1;
            }
            else if(i==4)
            {
                Str_Regression_Mid(1, (byte)(UpperData > 60 ? 60 : UpperData));
                Mid_StrLine_kb[0] = MidRegk * k;
                //Mid_StrLine_kb[1] = (L_black[1] + R_black[1]) / 2 - Mid_StrLine_kb[0] * 1;
                Mid_StrLine_kb[1] = MidRegb;
            }
            SetText_1("Mid_StrLine_kb[0] = " + Mid_StrLine_kb[0] + " Mid_StrLine_kb[1] = " + Mid_StrLine_kb[1]);
            SetText_1("//Meeting_BiasSeed_Find "+" LowerData: " + LowerData + " UpperData: " + UpperData);
            for (OlRow = LowerData; OlRow < UpperData && OlRow > 1&&OlRow<55; OlRow++)
            {
                x = (int)(Mid_StrLine_kb[0] * OlRow + Mid_StrLine_kb[1]);
                if (x < 0)
                    x = 0;
                else if (x > 185)
                    x = 185;
                Mask2[OlRow, x] = 1;
                //SetText_1("Find Point : " + " OlRow " + OlRow + " Line " + x);
                if (x == 0 || x == 185)
                {
                    SetText_1("Lost Car Bias Seed !");
                    return 0;
                }
                if (J_Pixels[OlRow][x] == black)
                {
                    if (J_Pixels[OlRow - 1][x] == black)
                    {
                        OlRow = (byte)(OlRow - 1);
                    }
                    Car_LeftMargin[0].row = Car_RightMargin[0].row = OlRow;
                    Car_LeftMargin[0].line = Car_RightMargin[0].line = (byte)x;
                    Car_LeftMargin[0].direction = (byte)NC_Direction.NC_downmiddle;
                    Car_RightMargin[0].direction = (byte)SC_Direction.SC_downmiddle;
                    Car_LeftLine.StartLine = Car_RightLine.StartLine = OlRow;
                    Mask1[OlRow, x] = 1;//标记
                    SetText_1("Car Bias Seed Find " + " OlRow: " + OlRow + " OlLine: " + x);
                    return 1;
                }
            }
            SetText_1("Lost Car Bias Seed !");
            SetText_1(" ");
            return 0;
        }
        void CarLineInfoType()
        {
            SetText_1("Car_LeftLine PointCnt:" + Car_LeftLine.PointCnt + "           Car_RightLine PointCnt:" + Car_RightLine.PointCnt);
            SetText_1("Car_LeftLine Agl_PointNum:" + Car_LeftLine.Agl_PointNum + "            Car_RightLine Agl_PointNum:" + Car_RightLine.Agl_PointNum);
            SetText_1("Car_LeftLine Agl_2_PointNum:" + Car_LeftLine.Agl_2_PointNum + "            Car_RightLine Agl_2_PointNum:" + Car_RightLine.Agl_2_PointNum);
            SetText_1("Car_LeftLine BroadWire_Cnt:" + Car_LeftLine.BroadWire_Cnt + "          Car_RightLine BroadWire_Cnt:" + Car_RightLine.BroadWire_Cnt);
            SetText_1("Car_LeftLine Straight_Cnt:" + Car_LeftLine.Straight_Cnt + "            Car_RightLine Straight_Cnt:" + Car_RightLine.Straight_Cnt);
            SetText_1("Car_LeftLine Turn_Cnt:" + Car_LeftLine.Turn_Cnt + "            Car_RightLine Turn_Cnt:" + Car_RightLine.Turn_Cnt);
            SetText_1("Car_LeftLine Reserve_Cnt:" + Car_LeftLine.Reserve_Cnt + "            Car_RightLine Reserve_Cnt:" + Car_RightLine.Reserve_Cnt);
            SetText_1("Car_LeftLine Rec_Point:" + Car_LeftLine.Rec_Point + "            Car_RightLine Rec_Point:" + Car_RightLine.Rec_Point);
            SetText_1("Car_LeftLine StartLine:" + Car_LeftLine.StartLine + "            Car_RightLine StartLine:" + Car_RightLine.StartLine);
            SetText_1("Car_LeftLine EndLine:" + Car_LeftLine.EndLine + "            Car_RightLine EndLine:" + Car_RightLine.EndLine);
        }
        byte Meeting_Judge() // 会车判断
        {
            if (MeetingFlag == 1 && RoadType != RoadTypeEnum.Meeting)
            {
                SetText_1("****************************Car Judge Start*************************");
                byte SeedFindCount = 0;
                if (RoadType == RoadTypeEnum.Turn_L
                    || ((RoadType == RoadTypeEnum.Circle_L || RoadType == RoadTypeEnum.Circle_R) && (Circle_State == Circle.Circle_3 || Circle_State == Circle.Circle_4 || Circle_State == Circle.Circle_5))
                    || RoadType == RoadTypeEnum.Turn_R)
                {
                    SeedFindCount = 3;
                }
                else
                {
                    SeedFindCount = 1;
                }
                for (byte cnt = 0; cnt < SeedFindCount; cnt++)
                {
                    switch (cnt)
                    {
                        case 0:
                            if (RoadType != RoadTypeEnum.Slope)
                            {
                                Meeting_BiasSeed_Find(2, (byte)(Scan_EndRow - 1), 4, 1.2f);
                            }
                            else
                            {
                                Car_Seed_Find(93, 2, 50);
                            }
                            break;
                        case 1:
                            CarLine_Init();
                            if (RoadType == RoadTypeEnum.Turn_L || (RoadType == RoadTypeEnum.Circle_L))
                            {
                                Meeting_BiasSeed_Find(2, (byte)(Scan_EndRow - 1), 1, 0.7f);
                            }
                            else if (RoadType == RoadTypeEnum.Turn_R || (RoadType == RoadTypeEnum.Circle_R))
                            {
                                Meeting_BiasSeed_Find(2, (byte)(Scan_EndRow - 1), 0, 0.7f);
                            }
                            break;
                        case 2:
                            CarLine_Init();
                            if (RoadType == RoadTypeEnum.Turn_L || (RoadType == RoadTypeEnum.Circle_L))
                            {
                                Meeting_BiasSeed_Find(2, (byte)(Scan_EndRow - 1), 1, 1.0f);
                            }
                            else if (RoadType == RoadTypeEnum.Turn_R || (RoadType == RoadTypeEnum.Circle_R))
                            {
                                Meeting_BiasSeed_Find(2, (byte)(Scan_EndRow - 1), 0, 1.0f);
                            }
                            break;
                        default: break;
                    }
                    Car_RegionScanLine(Scan_LineType.LeftType, 0, 50);
                    Car_RegionScanLine(Scan_LineType.RightType, 0, 50);
                    Traverse_CarAgl(Scan_LineType.LeftType, 69);
                    Traverse_CarAgl(Scan_LineType.RightType, 69);
                    CarLineInfoType();
                    if (Car_LeftLine.Error == 0 && Car_RightLine.Error == 0)
                    {
                        if (Car_RightLine.Agl_Row != 0 && Car_LeftLine.Agl_Row != 0
                            && Car_RightLine.Agl_2_Row != 0 && Car_LeftLine.Agl_2_Row != 0
                            && (float)Car_LeftLine.Straight_Cnt / (Car_LeftLine.Agl_2_PointNum - Car_LeftLine.Agl_PointNum) >= 0.3
                            && (float)Car_RightLine.Straight_Cnt / (Car_RightLine.Agl_2_PointNum - Car_RightLine.Agl_PointNum) >= 0.3
                            && (float)Car_RightLine.BroadWire_Cnt / Car_RightLine.Agl_PointNum >= 0.6
                            && (float)Car_LeftLine.BroadWire_Cnt / Car_LeftLine.Agl_PointNum >= 0.6
                            && my_fabs(Car_LeftLine.Agl_Row - Car_RightLine.Agl_Row) <= 30
                            && Car_widthCale(MeetingMode) == 1
                            )
                        {
                            CarTrailRow = (byte)((Car_RightLine.Agl_Row + Car_LeftLine.Agl_Row) / 2);
                            CarTrailLine = (byte)((Car_RightLine.Agl_Line + Car_LeftLine.Agl_Line) / 2);
                            RoadType = RoadTypeEnum.Meeting;
                            Meeting_State = Meeting_st.Meeting_0;
                            SetText_1("Start Meeting");
                            SetText_1("Trail Row = " + CarTrailRow + " Trail Line " + CarTrailLine);
                            setText用户自定义("Start Meeting");
                            return 1;
                        }
                        else if (Car_LeftLine.Agl_Row != 0 && Car_LeftLine.Agl_Row < 50
                            && Car_LeftLine.Agl_2_Row != 0
                            && (float)Car_LeftLine.BroadWire_Cnt / Car_LeftLine.Agl_PointNum >= 0.3
                            && (float)Car_LeftLine.Straight_Cnt / (Car_LeftLine.Agl_2_PointNum - Car_LeftLine.Agl_PointNum) >= 0.3
                            && Car_LeftLine.Turn_Cnt < 10

                            && Car_RightLine.Agl_Row == 0
                            && Car_RightLine.PointCnt < 30
                            && (float)Car_RightLine.BroadWire_Cnt / Car_RightLine.PointCnt >= 0.5
                            && Car_widthCale(MeetingMode) == 1
                            )
                        {
                            CarTrailRow = (byte)((Car_RightMargin[Car_RightLine.PointCnt].row + Car_LeftLine.Agl_Row) / 2);
                            CarTrailLine = (byte)((Car_RightMargin[Car_RightLine.PointCnt].line + Car_LeftLine.Agl_Line) / 2);
                            RoadType = RoadTypeEnum.Meeting;
                            Meeting_State = Meeting_st.Meeting_0;
                            SetText_1("Start Meeting");
                            SetText_1("Trail Row = " + CarTrailRow + " Trail Line " + CarTrailLine);
                            setText用户自定义("Start Meeting");
                            return 1;
                        }
                        else if (Car_RightLine.Agl_Row != 0 && Car_RightLine.Agl_Row < 50
                            && Car_RightLine.Agl_2_Row != 0
                            && (float)Car_RightLine.BroadWire_Cnt / Car_RightLine.Agl_PointNum >= 0.3
                            && (float)Car_RightLine.Straight_Cnt / (Car_RightLine.Agl_2_PointNum - Car_RightLine.Agl_PointNum) >= 0.3
                            && Car_RightLine.Turn_Cnt < 10

                            && Car_LeftLine.Agl_Row == 0
                            && Car_LeftLine.PointCnt < 30
                            && (float)Car_LeftLine.BroadWire_Cnt / Car_LeftLine.PointCnt >= 0.5
                            && Car_widthCale(MeetingMode) == 1
                            )
                        {
                            CarTrailRow = (byte)((Car_LeftMargin[Car_LeftLine.PointCnt].row + Car_RightLine.Agl_Row) / 2);
                            CarTrailLine = (byte)((Car_LeftMargin[Car_LeftLine.PointCnt].line + Car_RightLine.Agl_Line) / 2);
                            RoadType = RoadTypeEnum.Meeting;
                            Meeting_State = Meeting_st.Meeting_0;
                            SetText_1("Start Meeting");
                            SetText_1("Trail Row = " + CarTrailRow + " Trail Line " + CarTrailLine);
                            setText用户自定义("Start Meeting");
                            return 1;
                        }
                    }
                    else
                    {
                        SetText_1("Wrong Meeting");
                        return 0;
                    }
                }
                SetText_1("****************************Car Judge End*************************");
            }
            return 0;
        }
        void Meeting_Deal() // 会车赛道边线处理
        {
            SetText_1(" ");
            SetText_1("****************************Meeting Deal Start*************************");
            int x;
            if (Meeting_State == Meeting_st.Meeting_0 && CarTrailRow <= 30)
            {
                Meeting_State = Meeting_st.Meeting_1;
            }
            else if (Meeting_State == Meeting_st.Meeting_1 && CarTrailRow <= 10)
            {
                Meeting_State = Meeting_st.Meeting_2;
                //Meeting_Fps = Meeting_st3_Fps;
            }
            else if (Meeting_State == Meeting_st.Meeting_2 && CarTrailLine == 93 && CarTrailRow == 1)
            {
                Meeting_State = Meeting_st.Meeting_3;
            }
            switch (Meeting_State)
            {
                case Meeting_st.Meeting_0:
                case Meeting_st.Meeting_1:
                case Meeting_st.Meeting_2:
                    Str_LineParaCale_Mid(1, 93, CarTrailRow, CarTrailLine);
                    for (OlRow = 1; OlRow <= CarTrailRow; OlRow++)
                    {
                        x = (int)(Mid_StrLine_kb[0] * OlRow + Mid_StrLine_kb[1]);
                        if (x < 0)
                            x = 0;
                        else if (x > 185)
                            x = 185;
                        L_black[OlRow] = R_black[OlRow] = (byte)x;
                    }
                    Scan_EndRow = CarTrailRow;
                    break;
                case Meeting_st.Meeting_3:
                    for (OlRow = 1; OlRow <= 10; OlRow++)
                    {
                        L_black[OlRow] = 185;
                        R_black[OlRow] = 0;
                    }
                    Scan_EndRow = 10;
                    break;
            }
            SetText_1("****************************Meeting Deal Start*************************");
        }
        void Meeting_Complete()
        {
            Meeting_State = Meeting_st.Meeting_4;
        }
        #endregion
        #region 长直道道判断
        float LeftLine_k, RightLine_k,LeftStrError,RightStrError,MidError;
        void Straight_Judge() // 长直道判断
        {
            if ((RoadType == RoadTypeEnum.Common || RoadType == RoadTypeEnum.CrossLine) && Scan_EndRow > 50)
            {
                SetText_1(" ");
                SetText_1("****************************Straight Judge Start*************************");
                SetText_2("//Straight Judge");
                Str_Regression_Mid(1, (byte)(Scan_EndRow > 65 ? 65 : Scan_EndRow));
                MidError = Str_Quant(1, (byte)(Scan_EndRow > 65 ? 65 : Scan_EndRow), 2);
                SetText_2("Line Mid Reg k = " + MidRegk);
                SetText_2("Line Mid Error = " + MidError);
                //SetText_2("L60-R60 = "+ my_fabs(L_black[60] - R_black[60]));
                if (MidRegk < 0.5 && MidRegk > -0.5
                    && MidError <= 1
                    && (Slope_Fps == 0 || (Slope_Fps > 0 && L_black[50] - R_black[50] < 70))
                    )
                {
                    if (RoadType == RoadTypeEnum.Common 
                        //&& !((RightLine_1.RecStrError > 1 || LeftLine_1.RecStrError > 1)&&Scan_EndRow>=60&&my_fabs(L_black[60]-R_black[60])>40)
                        )
                    {
                        RoadType = RoadTypeEnum.Straight;
                    }
                    else if (RoadType == RoadTypeEnum.CrossLine)
                    {
                        CrossLineType = CrossLineTypeEnum.Long_Cross;
                    }
                }
                else
                {
                    if (RoadType == RoadTypeEnum.CrossLine)
                    {
                        CrossLineType = CrossLineTypeEnum.Short_Cross;
                    }
                }
                SetText_1("****************************Straight Judge End*************************");
            }
            else if (RoadType == RoadTypeEnum.Circle_L || RoadType == RoadTypeEnum.Circle_R)
            {
                if (Scan_EndRow > 50)
                {
                    SetText_1(" ");
                    SetText_1("****************************Circle Straight Judge Start*************************");
                    Str_Regression_Mid(1, (byte)(Scan_EndRow > 65 ? 65 : Scan_EndRow));
                    MidError = Str_Quant(1, (byte)(Scan_EndRow > 65 ? 65 : Scan_EndRow), 2);
                    SetText_2("Circle Line Mid Reg k = " + MidRegk);
                    SetText_2("Circle Line Mid Error = " + MidError);
                    if (MidRegk < 0.5 && MidRegk > -0.5)
                    {
                        if (MidError <= 1)
                            Circle_Road = Circle_RoadEnum.Circle_Straight;
                        else
                            Circle_Road = Circle_RoadEnum.Circle_Common;
                    }
                    else
                        Circle_Road = Circle_RoadEnum.Circle_Turn;
                    SetText_1("****************************Circle Straight Judge End*************************");
                }
                setText用户自定义("Circle_Road = " + Circle_Road);
            }
            if (RoadType == RoadTypeEnum.CrossLine)
            {
                setText用户自定义("CrossLineType = " + CrossLineType);
            }
        }
        #endregion
        #region 特殊情况赛道函数及变量
        void Common_Deal() // 普通赛道处理
        {
            if (RoadType == RoadTypeEnum.Common)
            {
                if (LeftLine_1.Agl_Row != 0 && LeftLine_1.Agl_2_Row != 0
                    && RightLine_1.Agl_Row != 0 && RightLine_1.Agl_2_Row != 0)
                {
                    Str_LineParaCale_Left(LeftLine_1.Agl_Row, LeftLine_1.Agl_Line, LeftLine_1.Agl_2_Row, LeftLine_1.Agl_2_Line);
                    Str_LineSet_Left(1, LeftLine_1.Agl_2_Row);
                    Str_LineParaCale_Right(RightLine_1.Agl_Row, RightLine_1.Agl_Line, RightLine_1.Agl_2_Row, RightLine_1.Agl_2_Line);
                    Str_LineSet_Right(1, RightLine_1.Agl_2_Row);
                }
            }
        }
        void Road_SpcJudge() // 对坡道、会车的判断
        {
            Slope_Judge();
            Meeting_Judge();
        }
        void Road_Deal() // 各赛道类型处理
        {
            if (RoadType == RoadTypeEnum.Slope)
                Slope_Deal();
            else if (RoadType == RoadTypeEnum.Meeting)
                Meeting_Deal();
            else if (RoadType == RoadTypeEnum.CrossLine)
                Cro_Deal();
            else if (RoadType == RoadTypeEnum.Circle_L || RoadType == RoadTypeEnum.Circle_R)
                Circle_Deal();
            else if (RoadType == RoadTypeEnum.Garage_L || RoadType == RoadTypeEnum.Garage_R)
                Garage_Deal();
            else if (RoadType == RoadTypeEnum.Out_Garage_L || RoadType == RoadTypeEnum.Out_Garage_R)
                Out_Garage_Deal();
            SetText_1("Left Bound Start Line = "+Left_Bound_StartLine+" Left Bound End Line = "+Left_Bound_EndLine);
            SetText_1("Right Bound Start Line = " + Right_Bound_StartLine + " Right Bound End Line = " + Right_Bound_EndLine);
            SetText_1("Scan End Line = " + Scan_EndRow); 
        }
        #endregion
        #region 透视变换函数及变量，未使用
        Trans_Marix Prst_Marix = new Trans_Marix();
        Trans_Marix[] Marix_Library = new Trans_Marix[15];
        Trans_Marix Temp_Marix = new Trans_Marix();
        Trans_Marix Marix_a = new Trans_Marix();
        Trans_Marix Marix_b = new Trans_Marix();
        point p1 = new point();
        point p2 = new point();
        point p3 = new point();
        point p4 = new point();
        point p5 = new point();
        point p6 = new point();
        point p7 = new point();
        point p8 = new point();
        point Trans_Point = new point();
        float Prst_offset = 0.8f;
        byte Prst_EndRow, Prst_L_EndRow, Prst_R_EndRow;
        byte Last_Middle = 93;
        float[] per_cm_Library = new float[15];
        float per_cm = 0.3261f;
        int Road_Width;
        int AngleZ_Correction = 0;
        float Trans_k = 0.085f;
        void Set_Library()
        {
#if UpperComputer
            Marix_Library[0] = new Trans_Marix();
            Marix_Library[1] = new Trans_Marix();
            Marix_Library[2] = new Trans_Marix();
            Marix_Library[3] = new Trans_Marix();
            Marix_Library[4] = new Trans_Marix();
            Marix_Library[5] = new Trans_Marix();
            Marix_Library[6] = new Trans_Marix();
            Marix_Library[7] = new Trans_Marix();
            Marix_Library[8] = new Trans_Marix();
            Marix_Library[9] = new Trans_Marix();
            Marix_Library[10] = new Trans_Marix();
            Marix_Library[11] = new Trans_Marix();
            Marix_Library[12] = new Trans_Marix();
            Marix_Library[13] = new Trans_Marix();
            Marix_Library[14] = new Trans_Marix();
#endif
            //-0
            Marix_Library[0].x11 = 1; Marix_Library[0].x12 = 0; Marix_Library[0].x13 = 0;
            Marix_Library[0].x21 = -2.8392f; Marix_Library[0].x22 = 0.6863f; Marix_Library[0].x23 = -0.0314f;
            Marix_Library[0].x31 = 14.1961f; Marix_Library[0].x32 = 1.5686f; Marix_Library[0].x33 = 1.1569f;
            per_cm_Library[0] = 1.9560f;
            //-100
            Marix_Library[1].x11 = 1; Marix_Library[1].x12 = 0; Marix_Library[1].x13 = 0;
            Marix_Library[1].x21 = -2.8392f; Marix_Library[1].x22 = 0.6863f; Marix_Library[1].x23 = -0.0314f;
            Marix_Library[1].x31 = 14.1961f; Marix_Library[1].x32 = 1.5686f; Marix_Library[1].x33 = 1.1569f;
            per_cm_Library[1] = 1.4516f;
            //-200
            Marix_Library[2].x11 = 1; Marix_Library[2].x12 = 0; Marix_Library[2].x13 = 0;
            Marix_Library[2].x21 = -2.8392f; Marix_Library[2].x22 = 0.6863f; Marix_Library[2].x23 = -0.0314f;
            Marix_Library[2].x31 = 14.1961f; Marix_Library[2].x32 = 1.5686f; Marix_Library[2].x33 = 1.1569f;
            per_cm_Library[2] = 1.4516f;
            //-300
            Marix_Library[3].x11 = 1; Marix_Library[3].x12 = 0; Marix_Library[3].x13 = 0;
            Marix_Library[3].x21 = -2.8392f; Marix_Library[3].x22 = 0.6863f; Marix_Library[3].x23 = -0.0314f;
            Marix_Library[3].x31 = 14.1961f; Marix_Library[3].x32 = 1.5686f; Marix_Library[3].x33 = 1.1569f;
            per_cm_Library[3] = 1.4516f;
            //-400
            Marix_Library[4].x11 = 1; Marix_Library[4].x12 = 0; Marix_Library[4].x13 = 0;
            Marix_Library[4].x21 = -2.8392f; Marix_Library[4].x22 = 0.6863f; Marix_Library[4].x23 = -0.0314f;
            Marix_Library[4].x31 = 14.1961f; Marix_Library[4].x32 = 1.5686f; Marix_Library[4].x33 = 1.1569f;
            per_cm_Library[4] = 1.4516f;
            //-500
            Marix_Library[5].x11 = 1; Marix_Library[5].x12 = 0; Marix_Library[5].x13 = 0;
            Marix_Library[5].x21 = -2.4540f; Marix_Library[5].x22 = 0.2190f; Marix_Library[5].x23 = -0.0260f;
            Marix_Library[5].x31 = 12.2698f; Marix_Library[5].x32 = 3.9048f; Marix_Library[5].x33 = 1.1302f;
            per_cm_Library[5] = 1.4516f;
            //-600
            Marix_Library[6].x11 = 1; Marix_Library[6].x12 = 0; Marix_Library[6].x13 = 0;
            Marix_Library[6].x21 = -2.1070f; Marix_Library[6].x22 = 0.3239f; Marix_Library[6].x23 = -0.0225f;
            Marix_Library[6].x31 = 10.5352f; Marix_Library[6].x32 = 3.3803f; Marix_Library[6].x33 = 1.1127f;
            per_cm_Library[6] = 1.2857f;
            //-700
            Marix_Library[7].x11 = 1; Marix_Library[7].x12 = 0; Marix_Library[7].x13 = 0;
            Marix_Library[7].x21 = -1.8466f; Marix_Library[7].x22 = 0.4076f; Marix_Library[7].x23 = -0.0197f;
            Marix_Library[7].x31 = 9.2329f; Marix_Library[7].x32 = 2.9620f; Marix_Library[7].x33 = 1.0987f;
            per_cm_Library[7] = 1.1538f;
            //-800
            Marix_Library[8].x11 = 1; Marix_Library[8].x12 = 0; Marix_Library[8].x13 = 0;
            Marix_Library[8].x21 = -1.6161f; Marix_Library[8].x22 = 0.4759f; Marix_Library[8].x23 = -0.0175f;
            Marix_Library[8].x31 = 8.0804f; Marix_Library[8].x32 = 2.6207f; Marix_Library[8].x33 = 1.0873f;
            per_cm_Library[8] = 1.0465f;
            //-900
            Marix_Library[9].x11 = 1; Marix_Library[9].x12 = 0; Marix_Library[9].x13 = 0;
            Marix_Library[9].x21 = -1.4021f; Marix_Library[9].x22 = 0.5453f; Marix_Library[9].x23 = -0.0152f;
            Marix_Library[9].x31 = 7.0105f; Marix_Library[9].x32 = 2.2737f; Marix_Library[9].x33 = 1.0758f;
            per_cm_Library[9] = 0.9574f;
            //-1000
            Marix_Library[10].x11 = 1; Marix_Library[10].x12 = 0; Marix_Library[10].x13 = 0;
            Marix_Library[10].x21 = -1.2161f; Marix_Library[10].x22 = 0.6077f; Marix_Library[10].x23 = -0.0131f;
            Marix_Library[10].x31 = 6.0808f; Marix_Library[10].x32 = 1.9615f; Marix_Library[10].x33 = 1.0654f;
            per_cm_Library[10] = 0.8491f;
            //-1100
            Marix_Library[11].x11 = 1; Marix_Library[11].x12 = 0; Marix_Library[11].x13 = 0;
            Marix_Library[11].x21 = -1.0629f; Marix_Library[11].x22 = 0.6571f; Marix_Library[11].x23 = -0.0114f;
            Marix_Library[11].x31 = 5.3143f; Marix_Library[11].x32 = 1.7143f; Marix_Library[11].x33 = 1.0571f;
            per_cm_Library[11] = 0.6521f;
            //-1200
            Marix_Library[12].x11 = 1; Marix_Library[12].x12 = 0; Marix_Library[12].x13 = 0;
            Marix_Library[12].x21 = -0.9525f; Marix_Library[12].x22 = 0.6976f; Marix_Library[12].x23 = -0.0101f;
            Marix_Library[12].x31 = 4.7626f; Marix_Library[12].x32 = 1.5122f; Marix_Library[12].x33 = 1.0504f;
            per_cm_Library[12] = 0.5294f;
            //-1300
            Marix_Library[13].x11 = 1; Marix_Library[13].x12 = 0; Marix_Library[13].x13 = 0;
            Marix_Library[13].x21 = -0.7295f; Marix_Library[13].x22 = 0.7582f; Marix_Library[13].x23 = -0.0081f;
            Marix_Library[13].x31 = 3.6478f; Marix_Library[13].x32 = 1.2089f; Marix_Library[13].x33 = 1.0403f;
            per_cm_Library[13] = 0.4091f;
            //-1400
            Marix_Library[14].x11 = 1; Marix_Library[14].x12 = 0; Marix_Library[14].x13 = 0;
            Marix_Library[14].x21 = -0.6517f; Marix_Library[14].x22 = 0.7920f; Marix_Library[14].x23 = -0.0069f;
            Marix_Library[14].x31 = 3.2587f; Marix_Library[14].x32 = 1.0400f; Marix_Library[14].x33 = 1.0347f;
            per_cm_Library[14] = 0.3462f;
        }
        void Chose_Library()
        {
            float quant;
#if UpperComputer
            byte[] Index = new byte[2];
            float[] MsIndex = new float[2];
#else
             byte Index [2];
            float MsIndex [2];
#endif
            //量化
            //quant = -AngleZ * 0.01f;
            //quant = (-AngleZ + AngleZ_Correction) * 0.01f;
            quant = (L_black[1] - R_black[1]) * (Trans_k);
            if (quant < 0)
                quant = 0;
            else if (quant > 14)
                quant = 14;
            //隶属度
            if (quant >= 0 && quant < 1)
            {
                Index[0] = 0;
                Index[1] = 1;
                MsIndex[0] = -quant + 1.0f;
                MsIndex[1] = quant;
            }
            else if (quant >= 1 && quant < 2)
            {
                Index[0] = 1;
                Index[1] = 2;
                MsIndex[0] = -quant + 2;
                MsIndex[1] = quant - 1;
            }
            else if (quant >= 2 && quant < 3)
            {
                Index[0] = 2;
                Index[1] = 3;
                MsIndex[0] = -quant + 3;
                MsIndex[1] = quant - 2;
            }
            else if (quant >= 3 && quant < 4)
            {
                Index[0] = 3;
                Index[1] = 4;
                MsIndex[0] = -quant + 4;
                MsIndex[1] = quant - 3;
            }
            else if (quant >= 4 && quant < 5)
            {
                Index[0] = 4;
                Index[1] = 5;
                MsIndex[0] = -quant + 5;
                MsIndex[1] = quant - 4;
            }
            else if (quant >= 5 && quant < 6)
            {
                Index[0] = 5;
                Index[1] = 6;
                MsIndex[0] = -quant + 6;
                MsIndex[1] = quant - 5;
            }
            else if (quant >= 6 && quant < 7)
            {
                Index[0] = 6;
                Index[1] = 7;
                MsIndex[0] = -quant + 7;
                MsIndex[1] = quant - 6;
            }
            else if (quant >= 7 && quant < 8)
            {
                Index[0] = 7;
                Index[1] = 8;
                MsIndex[0] = -quant + 8;
                MsIndex[1] = quant - 7;
            }
            else if (quant >= 8 && quant < 9)
            {
                Index[0] = 8;
                Index[1] = 9;
                MsIndex[0] = -quant + 9;
                MsIndex[1] = quant - 8;
            }
            else if (quant >= 9 && quant < 10)
            {
                Index[0] = 9;
                Index[1] = 10;
                MsIndex[0] = -quant + 10;
                MsIndex[1] = quant - 9;
            }
            else if (quant >= 10 && quant < 11)
            {
                Index[0] = 10;
                Index[1] = 11;
                MsIndex[0] = -quant + 11;
                MsIndex[1] = quant - 10;
            }
            else if (quant >= 11 && quant < 12)
            {
                Index[0] = 11;
                Index[1] = 12;
                MsIndex[0] = -quant + 12;
                MsIndex[1] = quant - 11;
            }
            else if (quant >= 12 && quant < 13)
            {
                Index[0] = 12;
                Index[1] = 13;
                MsIndex[0] = -quant + 13;
                MsIndex[1] = quant - 12;
            }
            else if (quant >= 13 && quant <= 14)
            {
                Index[0] = 13;
                Index[1] = 14;
                MsIndex[0] = -quant + 14;
                MsIndex[1] = quant - 13;
            }
            Prst_Marix.x11 = Marix_Library[Index[0]].x11 * MsIndex[0] + Marix_Library[Index[1]].x11 * MsIndex[1];
            Prst_Marix.x12 = Marix_Library[Index[0]].x12 * MsIndex[0] + Marix_Library[Index[1]].x12 * MsIndex[1];
            Prst_Marix.x13 = Marix_Library[Index[0]].x13 * MsIndex[0] + Marix_Library[Index[1]].x13 * MsIndex[1];
            Prst_Marix.x21 = Marix_Library[Index[0]].x21 * MsIndex[0] + Marix_Library[Index[1]].x21 * MsIndex[1];
            Prst_Marix.x22 = Marix_Library[Index[0]].x22 * MsIndex[0] + Marix_Library[Index[1]].x22 * MsIndex[1];
            Prst_Marix.x23 = Marix_Library[Index[0]].x23 * MsIndex[0] + Marix_Library[Index[1]].x23 * MsIndex[1];
            Prst_Marix.x31 = Marix_Library[Index[0]].x31 * MsIndex[0] + Marix_Library[Index[1]].x31 * MsIndex[1];
            Prst_Marix.x32 = Marix_Library[Index[0]].x32 * MsIndex[0] + Marix_Library[Index[1]].x32 * MsIndex[1];
            Prst_Marix.x33 = Marix_Library[Index[0]].x33 * MsIndex[0] + Marix_Library[Index[1]].x33 * MsIndex[1];
            per_cm = per_cm_Library[Index[0]] * MsIndex[0] + per_cm_Library[Index[1]] * MsIndex[1];
        }
        int find_point_L(int tgt_num)
        {
            int i = tgt_num;
            while (L_Trans[--i] == 185 && i > 0) ;
            return i;
        }
        void point_set_L(int bottom_point, int top_point)
        {
            int e, offset;
            e = top_point - bottom_point - 1;
            if (e == 0)
                return;
            else
            {
                if (e % 2 == 1)
                {
                    offset = e / 2 + 1;
                    //Temp = (L_Trans[bottom_point] + L_Trans[top_point]) / 2;
                    //if(Temp-(int)Temp>0.5)
                    //    Temp
                    L_Trans[bottom_point + offset] = (byte)((L_Trans[bottom_point] + L_Trans[top_point]) / 2);
                    point_set_L(bottom_point, bottom_point + offset);
                    point_set_L(bottom_point + offset, top_point);
                }
                else
                {
                    offset = e / 2;
                    L_Trans[bottom_point + offset] = (byte)((L_Trans[bottom_point] * 2 + L_Trans[top_point]) / 3);
                    L_Trans[bottom_point + offset + 1] = (byte)((L_Trans[bottom_point] + L_Trans[top_point] * 2) / 3);
                    point_set_L(bottom_point, bottom_point + offset);
                    point_set_L(bottom_point + offset + 1, top_point);
                }
            }
        }
        void inter_point_L(int tgt_num)
        {
            if (tgt_num > 0)
            {
                int start_num = find_point_L(tgt_num);
                point_set_L(start_num, tgt_num);
            }
        }
        int find_point_R(int tgt_num)
        {
            int i = tgt_num;
            while (R_Trans[--i] == 0 && i > 0) ;
            return i;
        }
        void point_set_R(int bottom_point, int top_point)
        {
            int e, offset;
            e = top_point - bottom_point - 1;
            if (e == 0)
                return;
            else
            {
                if (e % 2 == 1)
                {
                    offset = e / 2 + 1;
                    R_Trans[bottom_point + offset] = (byte)((R_Trans[bottom_point] + R_Trans[top_point]) / 2);
                    //SetText_1("R_Trans["+(bottom_point + offset)+"] = (R_Trans["+bottom_point+"] + R_Trans["+top_point+"]) / 2) = " + R_Trans[bottom_point + offset]);
                    point_set_R(bottom_point, bottom_point + offset);
                    point_set_R(bottom_point + offset, top_point);
                }
                else
                {
                    offset = e / 2;
                    R_Trans[bottom_point + offset] = (byte)((R_Trans[bottom_point] * 2 + R_Trans[top_point]) / 3);
                    //SetText_1("R_Trans[" + (bottom_point + offset) + "] = (R_Trans[" + bottom_point + "]*2 + R_Trans[" + top_point + "]) / 2) = " + R_Trans[bottom_point + offset]);
                    R_Trans[bottom_point + offset + 1] = (byte)((R_Trans[bottom_point] + R_Trans[top_point] * 2) / 3);
                    //SetText_1("R_Trans[" + (bottom_point + offset+1) + "] = (R_Trans[" + bottom_point + "] + R_Trans[" + top_point + "]*2) / 2) = " + R_Trans[bottom_point + offset+1]);
                    point_set_R(bottom_point, bottom_point + offset);
                    point_set_R(bottom_point + offset + 1, top_point);
                }
            }
        }
        void inter_point_R(int tgt_num)
        {
            if (tgt_num > 0)
            {
                int start_num = find_point_R(tgt_num);
                point_set_R(start_num, tgt_num);
            }
        }
        void Prst_SquareToQuadrilatera_a()
        {

            Marix_a.x33 = 1.0f;

            float dx3 = p1.x - p2.x + p3.x - p4.x;
            float dy3 = p1.y - p2.y + p3.y - p4.y;

            if (dx3 == 0 && dy3 == 0)
            {
                Marix_a.x11 = p2.x - p1.x;
                Marix_a.x21 = p3.x - p2.x;
                Marix_a.x31 = p1.x;
                Marix_a.x12 = p2.y - p1.y;
                Marix_a.x22 = p3.y - p2.y;
                Marix_a.x32 = p1.y;
                Marix_a.x13 = Marix_a.x23 = 0;
            }
            else
            {
                float dx1 = p2.x - p3.x;
                float dx2 = p4.x - p3.x;
                float dy1 = p2.y - p3.y;
                float dy2 = p4.y - p3.y;
                float denominator = dx1 * dy2 - dx2 * dy1;

                Marix_a.x13 = (dx3 * dy2 - dx2 * dy3) / denominator;
                Marix_a.x23 = (dx1 * dy3 - dx3 * dy1) / denominator;

                Marix_a.x11 = p2.x - p1.x + Marix_a.x13 * p2.x;
                Marix_a.x21 = p4.x - p1.x + Marix_a.x23 * p4.x;
                Marix_a.x31 = p1.x;
                Marix_a.x12 = p2.y - p1.y + Marix_a.x13 * p2.y;
                Marix_a.x22 = p4.y - p1.y + Marix_a.x23 * p4.y;
                Marix_a.x32 = p1.y;
            }
        }
        void Prst_SquareToQuadrilatera_b()
        {

            Marix_b.x33 = 1.0f;

            float dx3 = p5.x - p6.x + p7.x - p8.x;
            float dy3 = p5.y - p6.y + p7.y - p8.y;

            if (dx3 == 0 && dy3 == 0)
            {
                Marix_b.x11 = p6.x - p5.x;
                Marix_b.x21 = p7.x - p6.x;
                Marix_b.x31 = p5.x;
                Marix_b.x12 = p6.y - p5.y;
                Marix_b.x22 = p7.y - p6.y;
                Marix_b.x32 = p5.y;
                Marix_b.x13 = Marix_b.x23 = 0;
            }
            else
            {
                float dx1 = p6.x - p7.x;
                float dx2 = p8.x - p7.x;
                float dy1 = p6.y - p7.y;
                float dy2 = p8.y - p7.y;
                float denominator = dx1 * dy2 - dx2 * dy1;

                Marix_b.x13 = (dx3 * dy2 - dx2 * dy3) / denominator;
                Marix_b.x23 = (dx1 * dy3 - dx3 * dy1) / denominator;

                Marix_b.x11 = p6.x - p5.x + Marix_b.x13 * p6.x;
                Marix_b.x21 = p8.x - p5.x + Marix_b.x23 * p8.x;
                Marix_b.x31 = p5.x;
                Marix_b.x12 = p6.y - p5.y + Marix_b.x13 * p6.y;
                Marix_b.x22 = p8.y - p5.y + Marix_b.x23 * p8.y;
                Marix_b.x32 = p5.y;
            }
        }
        byte Prst_MatrixReverse()
        {
            float hls =
                Marix_a.x11 * (Marix_a.x22 * Marix_a.x33 - Marix_a.x23 * Marix_a.x32) -
                Marix_a.x21 * (Marix_a.x12 * Marix_a.x33 - Marix_a.x13 * Marix_a.x32) +
                Marix_a.x31 * (Marix_a.x12 * Marix_a.x23 - Marix_a.x13 * Marix_a.x22);

            if (hls != 0.0f)
            {
                float k = 1.0f / hls;

                Temp_Marix.x11 = k * (Marix_a.x22 * Marix_a.x33 - Marix_a.x23 * Marix_a.x32);
                Temp_Marix.x12 = k * (Marix_a.x13 * Marix_a.x32 - Marix_a.x12 * Marix_a.x33);
                Temp_Marix.x13 = k * (Marix_a.x12 * Marix_a.x23 - Marix_a.x13 * Marix_a.x22);

                Temp_Marix.x21 = k * (Marix_a.x23 * Marix_a.x31 - Marix_a.x21 * Marix_a.x33);
                Temp_Marix.x22 = k * (Marix_a.x11 * Marix_a.x33 - Marix_a.x13 * Marix_a.x31);
                Temp_Marix.x23 = k * (Marix_a.x21 * Marix_a.x13 - Marix_a.x11 * Marix_a.x23);

                Temp_Marix.x31 = k * (Marix_a.x21 * Marix_a.x32 - Marix_a.x22 * Marix_a.x31);
                Temp_Marix.x32 = k * (Marix_a.x12 * Marix_a.x31 - Marix_a.x11 * Marix_a.x32);
                Temp_Marix.x33 = k * (Marix_a.x11 * Marix_a.x22 - Marix_a.x21 * Marix_a.x12);

                return 1;
            }
            else return 0;
        }
        void Prst_MatrixMultiple()
        {
            Prst_Marix.x11 = Temp_Marix.x11 * Marix_b.x11 + Temp_Marix.x12 * Marix_b.x21 + Temp_Marix.x13 * Marix_b.x31;
            Prst_Marix.x12 = Temp_Marix.x11 * Marix_b.x12 + Temp_Marix.x12 * Marix_b.x22 + Temp_Marix.x13 * Marix_b.x32;
            Prst_Marix.x13 = Temp_Marix.x11 * Marix_b.x13 + Temp_Marix.x12 * Marix_b.x23 + Temp_Marix.x13 * Marix_b.x33;

            Prst_Marix.x21 = Temp_Marix.x21 * Marix_b.x11 + Temp_Marix.x22 * Marix_b.x21 + Temp_Marix.x23 * Marix_b.x31;
            Prst_Marix.x22 = Temp_Marix.x21 * Marix_b.x12 + Temp_Marix.x22 * Marix_b.x22 + Temp_Marix.x23 * Marix_b.x32;
            Prst_Marix.x23 = Temp_Marix.x21 * Marix_b.x13 + Temp_Marix.x22 * Marix_b.x23 + Temp_Marix.x23 * Marix_b.x33;

            Prst_Marix.x31 = Temp_Marix.x31 * Marix_b.x11 + Temp_Marix.x32 * Marix_b.x21 + Temp_Marix.x33 * Marix_b.x31;
            Prst_Marix.x32 = Temp_Marix.x31 * Marix_b.x12 + Temp_Marix.x32 * Marix_b.x22 + Temp_Marix.x33 * Marix_b.x32;
            Prst_Marix.x33 = Temp_Marix.x31 * Marix_b.x13 + Temp_Marix.x32 * Marix_b.x23 + Temp_Marix.x33 * Marix_b.x33;
        }
        point PrstTrans_Cale(int y, int x)
        {
#if UpperComputer
            point p_new = new point();
#else
            point p_new;
#endif
            float Temp_x_1, Temp_y_1, Temp_x_2, Temp_y_2;
            Temp_x_1 = x;
            Temp_y_1 = y;
            float z = Prst_Marix.x13 * Temp_x_1 + Prst_Marix.x23 * Temp_y_1 + Prst_Marix.x33;
            Temp_x_2 = (Prst_Marix.x11 * Temp_x_1 + Prst_Marix.x21 * Temp_y_1 + Prst_Marix.x31) / z;
            Temp_y_2 = (Prst_Marix.x12 * Temp_x_1 + Prst_Marix.x22 * Temp_y_1 + Prst_Marix.x32) / z;
            Temp_x_2 = Temp_x_2 * Prst_offset + 93 * (1 - Prst_offset);
            Temp_y_2 *= Prst_offset;
            if (Temp_x_2 > 185)
            {
                Temp_x_2 = 185;
            }
            else if (Temp_x_2 < 0)
            {
                Temp_x_2 = 0;
            }
            p_new.x = (int)Temp_x_2;
            p_new.y = (int)Temp_y_2;
            return p_new;
        }
        void Set_p1(int x, int y)
        {
            p1.x = x;
            p1.y = y;
        }
        void Set_p2(int x, int y)
        {
            p2.x = x;
            p2.y = y;
        }
        void Set_p3(int x, int y)
        {
            p3.x = x;
            p3.y = y;
        }
        void Set_p4(int x, int y)
        {
            p4.x = x;
            p4.y = y;
        }
        void Set_p5(int x, int y)
        {
            p5.x = x;
            p5.y = y;
        }
        void Set_p6(int x, int y)
        {
            p6.x = x;
            p6.y = y;
        }
        void Set_p7(int x, int y)
        {
            p7.x = x;
            p7.y = y;
        }
        void Set_p8(int x, int y)
        {
            p8.x = x;
            p8.y = y;
        }
        void Set_Prst_Matrix(float x11, float x12, float x13, float x21, float x22, float x23, float x31, float x32, float x33)
        {
            Prst_Marix.x11 = x11;
            Prst_Marix.x12 = x12;
            Prst_Marix.x13 = x13;
            Prst_Marix.x21 = x21;
            Prst_Marix.x22 = x22;
            Prst_Marix.x23 = x23;
            Prst_Marix.x31 = x31;
            Prst_Marix.x32 = x32;
            Prst_Marix.x33 = x33;
        }
        void Get_Prst_Matrix()
        {
            Prst_SquareToQuadrilatera_a();
            Prst_SquareToQuadrilatera_b();
            Prst_MatrixReverse();
            Prst_MatrixMultiple();
        }
        void per_cm_Cale()
        {
            if (!(L_Trans[1] == 185 || R_Trans[1] == 0))
            {
                //取第1行为赛道长度
                Road_Width = L_Trans[1] - R_Trans[1];
                //算出每像素点对应的实际长度
                per_cm = 45.0f / Road_Width;
            }
            SetText_1("Road_Width " + Road_Width + " = 45 cm . pem_cm = " + per_cm);
        }
        void Prst_Information_Set()
        {
            //获得矩阵
            //Set_p1(L_black[5], 5);
            //Set_p2(R_black[5], 5);
            //Set_p3(L_black[40], 40);
            //Set_p4(R_black[40], 40);
            //Set_p5(L_black[5], 5);
            //Set_p6(R_black[5], 5);
            //Set_p7(L_black[5], 40);
            //Set_p8(R_black[5], 40);
            //Get_Prst_Matrix();
            //设置矩阵
            Set_Prst_Matrix(1, 0, 0,
                       -1.2948f, 0.4033f, -0.0149f,
                        6.4741f, 2.9836f, 1.0746f);
            //从矩阵库中选取矩阵
            //Set_Library();
            //Chose_Library();
            //打印矩阵
            SetText_1("A11 A12 A13 " + Prst_Marix.x11 + " " + Prst_Marix.x12 + " " + Prst_Marix.x13);
            SetText_1("A21 A22 A23 " + Prst_Marix.x21 + " " + Prst_Marix.x22 + " " + Prst_Marix.x23);
            SetText_1("A31 A32 A33 " + Prst_Marix.x31 + " " + Prst_Marix.x32 + " " + Prst_Marix.x33);
        }
        void Prst_offset_Set()
        {
            float z, Temp_y;
            float Temp_y_1 = 60;
            z = Prst_Marix.x23 * Temp_y_1 + Prst_Marix.x33;
            Temp_y = (Prst_Marix.x22 * Temp_y_1 + Prst_Marix.x32) / z;
            //SetText_1("Temp_y = " + Temp_y);
            Prst_offset = 69.0f / Temp_y;
            if (Prst_offset < 0.5f)
                Prst_offset = 0.5f;
            else if (Prst_offset > 1)
                Prst_offset = 1.0f;
            SetText_1("Prst_offset  = " + Prst_offset);
        }
        void Prst_PointTrans()
        {
            for (OlRow = 1; OlRow <= Scan_EndRow; OlRow++)
            {
                //SetText_1("OlRow = LeftLine_1.StartLine = "+ LeftLine_1.StartLine);
                if (L_black[OlRow] < 185)  //边界存在
                {
                    Trans_Point = PrstTrans_Cale(OlRow, L_black[OlRow]);
                    if (Trans_Point.y <= 69)  //边界行在69内
                    {
                        L_Trans[Trans_Point.y] = (byte)(Trans_Point.x);
                        inter_point_L(Trans_Point.y);
                    }
                    else  //边界行超过了69，退出并记录结束行
                    {
                        Trans_Point = PrstTrans_Cale((byte)(OlRow - 1), L_black[OlRow - 1]);
                        Prst_L_EndRow = (byte)Trans_Point.y;
                        break;
                    }
                }
                else if (OlRow != 1) //边界不存在，退出并记录结束行
                {
                    Trans_Point = PrstTrans_Cale((byte)(OlRow - 1), L_black[OlRow - 1]);
                    Prst_L_EndRow = (byte)Trans_Point.y;
                    break;
                }
                else
                {
                    Prst_L_EndRow = 0;
                    break;
                }
            }
            if (Prst_L_EndRow == 69)  //正常结束变换，把最后一个点的行数作为结束行
            {
                Prst_L_EndRow = (byte)Trans_Point.y;
            }
            for (OlRow = 1; OlRow <= Scan_EndRow; OlRow++)
            {
                //SetText_1("OlRow = RightLine_1.StartLine = " + RightLine_1.StartLine);
                if (R_black[OlRow] > 0)
                {
                    Trans_Point = PrstTrans_Cale(OlRow, R_black[OlRow]);
                    if (Trans_Point.y <= 69)
                    {
                        R_Trans[Trans_Point.y] = (byte)(Trans_Point.x);
                        //SetText_1("R_Trans["+Trans_Point.y+"] = (Trans_Point.x + 93 * (1 - offset) = "+ R_Trans[Trans_Point.y]);
                        inter_point_R(Trans_Point.y);
                    }
                    else
                    {
                        Trans_Point = PrstTrans_Cale((byte)(OlRow - 1), R_black[OlRow - 1]);
                        Prst_R_EndRow = (byte)Trans_Point.y;
                        break;
                    }
                }
                else if (OlRow != 1)
                {
                    Trans_Point = PrstTrans_Cale((byte)(OlRow - 1), R_black[OlRow - 1]);
                    Prst_R_EndRow = (byte)Trans_Point.y;
                    break;
                }
                else
                {
                    Prst_R_EndRow = 0;
                    break;
                }
            }
            if (Prst_R_EndRow == 69)
            {
                Prst_R_EndRow = (byte)Trans_Point.y;
            }
            Prst_EndRow = Prst_R_EndRow > Prst_L_EndRow ? Prst_R_EndRow : Prst_L_EndRow;
            SetText_1("Prst_L_EndRow " + Prst_L_EndRow);
            SetText_1("Prst_R_EndRow " + Prst_R_EndRow);
            SetText_1("Prst_EndRow " + Prst_EndRow);
        }
        void Prst_Trans()
        {
            SetText_1(" ");
            SetText_1("****************************Prst Trans Start*************************");
            Prst_EndRow = Prst_R_EndRow = Prst_L_EndRow = 69;
            Prst_Information_Set();
            //Prst_offset_Set();
            Prst_PointTrans();
            //per_cm_Cale();
            SetText_1("****************************Prst Trans End*************************");
        }
        #endregion
        #region 中线计算函数及变量
        enum Mid_type { Common_Mid, Prst_Mid };
        enum Mid_offset_type
        {
            No_offset,
            Left_offset,
            Right_offset,
        }
        Mid_offset_type Mid_offset_Type;
        void MidLine_Set(Mid_type Mid_Type)
        {
            byte Mid_EndRow, Left_EndRow, Right_EndRow = 0;
            int Temp;
            if (Mid_Type == Mid_type.Prst_Mid)
            {
                Left_EndRow = Prst_L_EndRow;
                Right_EndRow = Prst_R_EndRow;
                if (Right_EndRow > Left_EndRow)
                {
                    Mid_offset_Type = Mid_offset_type.Left_offset;
                    Mid_EndRow = Right_EndRow;
                }
                else
                {
                    Mid_offset_Type = Mid_offset_type.Right_offset;
                    Mid_EndRow = Left_EndRow;
                }
                if (Mid_offset_Type == Mid_offset_type.Left_offset)
                {
                    if (Left_EndRow != 0)
                    {
                        for (x = 0; x <= Left_EndRow; x++)
                        {
                            LCenter[x] = (byte)((R_Trans[x] + L_Trans[x]) / 2);
                            //SetText_1("LCenter[" + x + "] = ((R_Trans[" + x + "] + L_Trans[" + x + "]) / 2) = " + LCenter[x]);
                        }
                        for (x = (byte)(Left_EndRow + 1); x <= Mid_EndRow; x++)
                        {
                            Temp = LCenter[x - 1] + R_Trans[x] - R_Trans[x - 1];
                            if (Temp < 0)
                                LCenter[x] = 0;
                            else if (Temp > 185)
                                LCenter[x] = 185;
                            else
                                LCenter[x] = (byte)Temp;
                            //SetText_1("LCenter[" + x + "] = (LCenter[" + (x - 1) + "] + R_Trans[" + x + "] - R_Trans[" + (x - 1) + "]) = " + LCenter[x]);
                        }
                    }
                    else
                    {
                        LCenter[0] = LCenter[1] = Last_Middle;
                        SetText_1("LCenter[0] = Last_Middle = " + Last_Middle);
                        for (x = 2; x <= Mid_EndRow; x++)
                        {
                            Temp = LCenter[x - 1] + R_Trans[x] - R_Trans[x - 1];
                            if (Temp < 0)
                                LCenter[x] = 0;
                            else if (Temp > 185)
                                LCenter[x] = 185;
                            else
                                LCenter[x] = (byte)Temp;
                            //SetText_1("LCenter[" + x + "] = LCenter[" + (x - 1) + "] + R_Trans[" + x + "] - R_Trans[" + (x - 1) + "] = " + LCenter[x]);
                        }
                    }
                }
                else
                {
                    if (Right_EndRow != 0)
                    {
                        for (x = 0; x <= Right_EndRow; x++)
                        {
                            LCenter[x] = (byte)((R_Trans[x] + L_Trans[x]) / 2);
                        }
                        for (x = (byte)(Right_EndRow + 1); x <= Mid_EndRow; x++)
                        {
                            Temp = LCenter[x - 1] + L_Trans[x] - L_Trans[x - 1];
                            if (Temp < 0)
                                LCenter[x] = 0;
                            else if (Temp > 185)
                                LCenter[x] = 185;
                            else
                                LCenter[x] = (byte)Temp;
                        }
                    }
                    else
                    {
                        LCenter[0] = LCenter[1] = Last_Middle;
                        SetText_1("LCenter[0] = Last_Middle = " + Last_Middle);
                        for (x = 2; x <= Mid_EndRow; x++)
                        {
                            Temp = LCenter[x - 1] + L_Trans[x] - L_Trans[x - 1];
                            if (Temp < 0)
                                LCenter[x] = 0;
                            else if (Temp > 185)
                                LCenter[x] = 185;
                            else
                                LCenter[x] = (byte)Temp;
                        }
                    }
                }
            }
            else
            {
                Left_EndRow = Left_Bound_EndLine;
                Right_EndRow = Right_Bound_EndLine;
                SetText_1("Right_Bound_EndLine = " + Right_Bound_EndLine);
                if (Right_EndRow > Left_EndRow)
                {
                    Mid_offset_Type = Mid_offset_type.Left_offset;
                    Mid_EndRow = Right_EndRow;
                }
                else
                {
                    Mid_offset_Type = Mid_offset_type.Right_offset;
                    Mid_EndRow = Left_EndRow;
                }
                if (Mid_offset_Type == Mid_offset_type.Left_offset)
                {
                    if (Left_EndRow != 0)
                    {
                        for (x = 0; x <= Left_EndRow; x++)
                        {
                            LCenter[x] = (byte)((R_black[x] + L_black[x]) / 2);
                            //SetText_1("LCenter[" + x + "] = ((R_black[" + x + "] + L_black[" + x + "]) / 2) = " + LCenter[x]);
                        }
                        for (x = (byte)(Left_EndRow + 1); x <= Mid_EndRow; x++)
                        {
                            Temp = LCenter[x - 1] + R_black[x] - R_black[x - 1];
                            if (Temp < 0)
                                LCenter[x] = 0;
                            else if (Temp > 185)
                                LCenter[x] = 185;
                            else
                                LCenter[x] = (byte)Temp;
                            //SetText_1("LCenter[" + x + "] = (LCenter[" + (x - 1) + "] + R_black[" + x + "] - R_black[" + (x - 1) + "]) = " + LCenter[x]);
                        }
                    }
                    else
                    {
                        LCenter[0] = LCenter[1] = Last_Middle;
                        SetText_1("LCenter[0] = Last_Middle = " + Last_Middle);
                        for (x = 2; x <= Mid_EndRow; x++)
                        {
                            Temp = LCenter[x - 1] + R_black[x] - R_black[x - 1];
                            if (Temp < 0)
                                LCenter[x] = 0;
                            else if (Temp > 185)
                                LCenter[x] = 185;
                            else
                                LCenter[x] = (byte)Temp;
                            //SetText_1("LCenter[" + x + "] = LCenter[" + (x - 1) + "] + R_black[" + x + "] - R_black[" + (x - 1) + "] = " + LCenter[x]);
                        }
                    }
                }
                else
                {
                    if (Right_EndRow != 0)
                    {
                        for (x = 0; x <= Right_EndRow; x++)
                        {
                            LCenter[x] = (byte)((R_black[x] + L_black[x]) / 2);
                        }
                        for (x = (byte)(Right_EndRow + 1); x <= Mid_EndRow; x++)
                        {
                            Temp = LCenter[x - 1] + L_black[x] - L_black[x - 1];
                            if (Temp < 0)
                                LCenter[x] = 0;
                            else if (Temp > 185)
                                LCenter[x] = 185;
                            else
                                LCenter[x] = (byte)Temp;
                        }
                    }
                    else
                    {
                        SetText_1("Right_EndRow = " + Right_EndRow);
                        LCenter[0] = LCenter[1] = Last_Middle;
                        SetText_1("LCenter[0] = Last_Middle = " + Last_Middle);
                        for (x = 2; x <= Mid_EndRow; x++)
                        {
                            Temp = LCenter[x - 1] + L_black[x] - L_black[x - 1];
                            if (Temp < 0)
                                LCenter[x] = 0;
                            else if (Temp > 185)
                                LCenter[x] = 185;
                            else
                                LCenter[x] = (byte)Temp;
                        }
                    }
                }
            }
            if (Right_EndRow != 0 && Left_EndRow != 0)
            {
                Last_Middle = LCenter[1];
                //SetText_1("Last_Middle = LCenter[1] = " + LCenter[1]);
            }
            SetText_1("Mid_offset_Type = " + Mid_offset_Type);
        }
        void MidLine_NoOffset_Set()
        {
            for (byte i = 0; i < Scan_EndRow; i++)
            {
                LCenter[i] = (byte)((L_black[i] + R_black[i]) / 2);
                //SetText_1(" LCenter["+i+"] = (L_black["+i+"] + R_black["+i+"]) / 2) = "+ " L_black[i] "+ " R_black[i] = "+ LCenter[i]);
            }
            if (Scan_EndRow==0)
            {
                Scan_EndRow = 1;
                LCenter[1] = LCenter[0] = 93;
            }
        }
        void SetInitSeedLine()
        {
            //InitSeedLine = 93;
            InitSeedLine = LCenter[1];
            if (InitSeedLine < 30)
                InitSeedLine = 30;
            else if (InitSeedLine > 155)
                InitSeedLine = 155;
        }
        #endregion
        #region 晃动状态判断
        byte Swag_Flag = 0;
        void Swag_Judge()
        {

        }
        void Swag_Scan()
        {

        }
        #endregion
        #region 弯道赛道信息计算函数及变量，未使用
        float R_radius, L_radius, Expt_radius;
        float Expt_AnguarSpeed;  //通过半径求预计角速度与实际角速度的插差值
        float d_AnguarSpeed;
        float Length_Cale(int y0, int x0, int y1, int x1)
        {
            int Temp_y, Temp_x;
            Temp_y = (y0 - y1);
            Temp_x = (x0 - x1);
            SetText_1("y0: " + y0 + " x0: " + x0 + " , " + " y1: " + y1 + " x1: " + x1);
            SetText_1("L = " + (float)Math.Sqrt(Temp_x * Temp_x + Temp_y * Temp_y));
            return (float)Math.Sqrt(Temp_x * Temp_x + Temp_y * Temp_y);
        }
        float Radius_Threepoint_Cale(int y0, int x0, int y1, int x1, int y2, int x2)
        {
            float a, b, c, r, numerator, denominator;
            a = Length_Cale(y0, x0, y1, x1);
            b = Length_Cale(y0, x0, y2, x2);
            c = Length_Cale(y1, x1, y2, x2);
            //SetText_1("a = " + a + " b = " + b + " c = " + c);
            numerator = a * b * c;
            //SetText_1("numerator = "+ numerator);
            denominator = (float)Math.Sqrt((a + b + c) * (a + b - c) * (a + c - b) * (b + c - a));
            //SetText_1("denominator = " + "Sqrt(" + (a + b + c) + " * " + (a + b - c) + " * " + (a + c - b) + " * " + (b + c - a) +" = "+ denominator);
            r = numerator / denominator;
            if (r > 1000)
                r = 1000;
            SetText_1(" r = " + r);
            return r;
        }
        void Raidus_Cale()
        {
            //通过三点求半径算出，左右边线0-20的圆的大概半径
            if (RoadType == RoadTypeEnum.Turn_L)
            {
                if (Prst_R_EndRow < 50)
                {
                    R_radius = Radius_Threepoint_Cale(2, R_Trans[2],
                                         Prst_R_EndRow / 2, R_Trans[Prst_R_EndRow / 2],
                                         Prst_R_EndRow, R_Trans[Prst_R_EndRow]);
                    R_radius = R_radius * per_cm;
                    //Expt_radius = R_radius - 22.5f;
                }
                else
                {
                    R_radius = Radius_Threepoint_Cale(2, R_Trans[2],
                                        25, R_Trans[20],
                                        50, R_Trans[40]);
                    R_radius = R_radius * per_cm;
                    //Expt_radius = R_radius - 22.5f;
                }
                Expt_radius = -R_radius;
                SetText_1("R_radius = " + R_radius + "cm");
                SetText_1("Expt_radius = " + Expt_radius + "cm");
            }
            else if (RoadType == RoadTypeEnum.Turn_R)
            {
                if (Prst_L_EndRow < 50)
                {
                    L_radius = Radius_Threepoint_Cale(2, L_Trans[2],
                                        Prst_L_EndRow / 2, L_Trans[Prst_L_EndRow / 2],
                                        Prst_L_EndRow, L_Trans[Prst_L_EndRow]);
                    L_radius = L_radius * per_cm;
                    //Expt_radius = L_radius - 22.5f;
                }
                else
                {
                    L_radius = Radius_Threepoint_Cale(2, L_Trans[2],
                                        25, L_Trans[20],
                                        50, L_Trans[40]);
                    L_radius = L_radius * per_cm;
                    //Expt_radius = L_radius - 22.5f;
                }
                Expt_radius = L_radius;
                SetText_1("L_radius = " + L_radius + "cm");
                SetText_1("Expt_radius = " + Expt_radius + "cm");
            }
        }
        void AnguarSpeed_Cale()
        {
            Raidus_Cale();
            //Actual_Speed = Encoder_Speed;
            //Actual_AnguarSpeed = Angle_Data.y_speed;
            Expt_AnguarSpeed = Actual_Speed / Expt_radius;
            d_AnguarSpeed = (Expt_AnguarSpeed - Actual_AnguarSpeed) * 100;
        }
        void Turn_Road_Deal()
        {
            SetText_1("");
            SetText_1("****************************Road Information Deal Start*************************");
            //d_MassCenter_Cale();
            AnguarSpeed_Cale();
            SetText_1("****************************Road Information Deal End*************************");
        }
        #endregion
        #region 普通赛道信息计算函数及变量，未使用
        int Camera_Middle_Error;
        int AngleLine;
        int[] Rule_AngleLine = new int[13] //打角行
{
// 	0  ,   100 , 200 , 300 , 400 , 500 , 600 , 700 , 800,900 ,1000,1100,1200
    5  ,   6 ,     7 ,   8 ,    9,   10,  11 ,  12 , 13 , 14 , 15 , 16 , 17
//	  0  , 0.05 , 0.11 , 0.148 , 0.26 , 0.31 , 0.41
};
#if UpperComputer
        int AngleLine_Get()
        {
            float quant;
            byte[] Index = new byte[2];
            float[] MsIndex = new float[2];
            //量化
            quant = Encoder_Cnt * 0.01f;
            if (quant < 0)
                quant = 0;
            else if (quant > 8)
                quant = 8;
            //隶属度
            if (quant >= 0 && quant < 1)
            {
                Index[0] = 0;
                Index[1] = 1;
                MsIndex[0] = -quant + 1.0f;
                MsIndex[1] = quant;
            }
            else if (quant >= 1 && quant < 2)
            {
                Index[0] = 1;
                Index[1] = 2;
                MsIndex[0] = -quant + 2;
                MsIndex[1] = quant - 1;
            }
            else if (quant >= 2 && quant < 3)
            {
                Index[0] = 2;
                Index[1] = 3;
                MsIndex[0] = -quant + 3;
                MsIndex[1] = quant - 2;
            }
            else if (quant >= 3 && quant < 4)
            {
                Index[0] = 3;
                Index[1] = 4;
                MsIndex[0] = -quant + 4;
                MsIndex[1] = quant - 3;
            }
            else if (quant >= 4 && quant < 5)
            {
                Index[0] = 4;
                Index[1] = 5;
                MsIndex[0] = -quant + 5;
                MsIndex[1] = quant - 4;
            }
            else if (quant >= 5 && quant < 6)
            {
                Index[0] = 5;
                Index[1] = 6;
                MsIndex[0] = -quant + 6;
                MsIndex[1] = quant - 5;
            }
            else if (quant >= 6 && quant < 7)
            {
                Index[0] = 6;
                Index[1] = 7;
                MsIndex[0] = -quant + 7;
                MsIndex[1] = quant - 6;
            }
            else if (quant >= 7 && quant < 8)
            {
                Index[0] = 7;
                Index[1] = 8;
                MsIndex[0] = -quant + 8;
                MsIndex[1] = quant - 7;
            }
            else if (quant >= 8 && quant < 9)
            {
                Index[0] = 8;
                Index[1] = 9;
                MsIndex[0] = -quant + 9;
                MsIndex[1] = quant - 8;
            }
            else if (quant >= 9 && quant < 10)
            {
                Index[0] = 9;
                Index[1] = 10;
                MsIndex[0] = -quant + 10;
                MsIndex[1] = quant - 9;
            }
            else if (quant >= 10 && quant < 11)
            {
                Index[0] = 10;
                Index[1] = 11;
                MsIndex[0] = -quant + 11;
                MsIndex[1] = quant - 10;
            }
            else if (quant >= 11 && quant <= 12)
            {
                Index[0] = 11;
                Index[1] = 12;
                MsIndex[0] = -quant + 12;
                MsIndex[1] = quant - 11;
            }
            AngleLine = (int)(Rule_AngleLine[Index[0]] * MsIndex[0] + Rule_AngleLine[Index[1]] * MsIndex[1]);
            if (AngleLine < Scan_EndRow)
            {
                //SetText_2("Simu AngleLine = " + AngleLine);
                return AngleLine;
            }
            else
            {
                //SetText_2("Simu AngleLine = Scan_EndRow =" + AngleLine);
                AngleLine = Scan_EndRow;
                return Scan_EndRow;
            }
        }

        void Common_Road_Deal()
        {
            Camera_Middle_Error = (LCenter[AngleLine_Get()] - 93);
            //SetText_2("Camera_Middle_Error = " + Camera_Middle_Error);
        }
#endif
        #endregion
        #region 切线计算函数及变量，未使用
        float Tang_Reg_K, Tang_Reg_b;
        float d_MassCenter;
        byte Tang_StartLine = 1;
        float[] ArcTan_New = new float[41]
{
            0.0f,0.099668652f,0.19739556f,0.291456794f,0.380506377f,0.463647609f,0.5404195f,0.610725964f,0.674740942f,0.732815102f,0.785398163f,0.832981267f,0.876058051f,0.915100701f,0.950546841f,0.982793723f,1.012197011f,1.03907226f,1.063697822f,1.086318398f,1.107148718f,1.126377117f,1.144168834f,1.160668986f,1.176005207f,1.19028995f,1.203622493f,1.216090675f,1.227772386f,1.238736859f,1.249045772f,1.258754205f,1.267911458f,1.276561762f,1.284744885f,1.292496668f,1.299849476f,1.306832603f,1.313472612f,1.31979364f,1.325817664f
};
        float[] Rule_TangentLine = new float[13]
{
// 	0  ,   100 , 200 , 300 , 400 , 500 , 600 , 700 , 800,900 ,1000,1100,1200
    5  ,   6 ,     7 ,   8 ,    9,   10,  11 ,  12 , 13 , 14 , 15 , 16 , 17
   //	  0  , 0.05 , 0.11 , 0.148 , 0.26 , 0.31 , 0.41
};
        float My_SheetArcTan_New(float x)
        {
            int Temp = (int)(x * 10);
            if (Temp > 40)
            {
                Temp = 40;
            }
            else if (Temp < -40)
            {
                Temp = -40;
            }
            if (Temp >= 0)
                return ArcTan_New[Temp];
            else
                return -ArcTan_New[-Temp];
        }
        void Regression_Tang(byte startline)

        {
            int i;
            float SumUp, SumDown, avrX, avrY, SumX = 0, SumY = 0, SumLines = 0;
            byte interval;
            byte endline = (byte)((startline + 5) > Prst_EndRow ? Prst_EndRow : (startline + 5));
            if (endline - startline >= 12)
                interval = 4;
            else if (endline - startline >= 6)
                interval = 2;
            else
                interval = 1;
            for (i = startline; i < endline; i++)
            {
                if (i % interval == 0)
                {
                    SumX += i;
                    SumY += LCenter[i];
                    SumLines++;
                }
            }
            avrX = SumX / SumLines;     //X的平均值
            avrY = SumY / SumLines;     //Y的平均值       
            SumUp = 0;
            SumDown = 0;
            for (i = startline; i < endline; i++)
            {
                if (i % interval == 0)
                {
                    SumUp += ((float)LCenter[i] - avrY) * (i - avrX);
                    SumDown += (i - avrX) * (i - avrX);
                }
            }
            if (SumDown == 0)
                Tang_Reg_K = 0;
            else
            {
                Tang_Reg_K = (float)(SumUp / SumDown);
                SetText_1("Tang_Reg_K " + Tang_Reg_K + " From " + startline + " To " + endline);
            }
            Tang_Reg_b = (SumY - Tang_Reg_K * SumX) / (float)SumLines;  //截距
        }
        byte Tang_Regress_Get(byte y)
        {
            float x = y * Tang_Reg_K + Tang_Reg_b;
            if (x > 183)
                x = 183;
            else if (x < 2)
                x = 2;
            return (byte)x;
        }
        void TangentLine_Set()
        {
            for (x = 0; x <= 20; x++)
            {
                Tang_Line[x] = Tang_Regress_Get(x);
            }
        }
#if UpperComputer
        int TangentLine_Get()
        {
            float quant;

            byte[] Index = new byte[2];
            float[] MsIndex = new float[2];

            int AngleLine;
            //量化
            quant = Encoder_Cnt * 0.01f;
            if (quant < 0)
                quant = 0;
            else if (quant > 8)
                quant = 8;
            //隶属度
            if (quant >= 0 && quant < 1)
            {
                Index[0] = 0;
                Index[1] = 1;
                MsIndex[0] = -quant + 1.0f;
                MsIndex[1] = quant;
            }
            else if (quant >= 1 && quant < 2)
            {
                Index[0] = 1;
                Index[1] = 2;
                MsIndex[0] = -quant + 2;
                MsIndex[1] = quant - 1;
            }
            else if (quant >= 2 && quant < 3)
            {
                Index[0] = 2;
                Index[1] = 3;
                MsIndex[0] = -quant + 3;
                MsIndex[1] = quant - 2;
            }
            else if (quant >= 3 && quant < 4)
            {
                Index[0] = 3;
                Index[1] = 4;
                MsIndex[0] = -quant + 4;
                MsIndex[1] = quant - 3;
            }
            else if (quant >= 4 && quant < 5)
            {
                Index[0] = 4;
                Index[1] = 5;
                MsIndex[0] = -quant + 5;
                MsIndex[1] = quant - 4;
            }
            else if (quant >= 5 && quant < 6)
            {
                Index[0] = 5;
                Index[1] = 6;
                MsIndex[0] = -quant + 6;
                MsIndex[1] = quant - 5;
            }
            else if (quant >= 6 && quant < 7)
            {
                Index[0] = 6;
                Index[1] = 7;
                MsIndex[0] = -quant + 7;
                MsIndex[1] = quant - 6;
            }
            else if (quant >= 7 && quant < 8)
            {
                Index[0] = 7;
                Index[1] = 8;
                MsIndex[0] = -quant + 8;
                MsIndex[1] = quant - 7;
            }
            else if (quant >= 8 && quant < 9)
            {
                Index[0] = 8;
                Index[1] = 9;
                MsIndex[0] = -quant + 9;
                MsIndex[1] = quant - 8;
            }
            else if (quant >= 9 && quant < 10)
            {
                Index[0] = 9;
                Index[1] = 10;
                MsIndex[0] = -quant + 10;
                MsIndex[1] = quant - 9;
            }
            else if (quant >= 10 && quant < 11)
            {
                Index[0] = 10;
                Index[1] = 11;
                MsIndex[0] = -quant + 11;
                MsIndex[1] = quant - 10;
            }
            else if (quant >= 11 && quant <= 12)
            {
                Index[0] = 11;
                Index[1] = 12;
                MsIndex[0] = -quant + 12;
                MsIndex[1] = quant - 11;
            }
            AngleLine = (int)(Rule_TangentLine[Index[0]] * MsIndex[0] + Rule_TangentLine[Index[1]] * MsIndex[1]);
            if (AngleLine < Scan_EndRow)
            {
                //SetText_2("Rule_TangentLine = " + AngleLine);
                return AngleLine;
            }
            else
            {
                //SetText_2("Rule_TangentLine = Scan_EndRow =" + AngleLine);
                return Scan_EndRow;
            }
        }
#endif
        void d_MassCenter_Cale()
        {
            Regression_Tang(1);
            //Regression_Tang((byte)TangentLine_Get());
            //d_MassCenter = My_SheetArcTan_New(Tang_Reg_K);
            d_MassCenter = (float)Math.Atan(Tang_Reg_K);
            //SetText_2(" K " + Tang_Reg_K + " d_MassCenter " + d_MassCenter);
#if UpperComputer
            TangentLine_Set();
#endif
        }

        #endregion
        #region 模糊控制函数及变量，未使用
        FuzPID Angular_FuzPID = new FuzPID();
        FuzPID MassCenter_FuzPID = new FuzPID();
        enum Turn_PID_type { Turn_Common, Turn_Prst };
        Turn_PID_type Turn_PID_Type;
        enum Fuz
        {
            NB = -6,
            NM = -4,
            NS = -2,
            ZO = 0,
            PS = 2,
            PM = 4,
            PB = 6,
        };
        Fuz[,] RuleP = new Fuz[7, 7]
        {
          { Fuz.PB , Fuz.PB , Fuz.PM , Fuz.PS , Fuz.ZO , Fuz.NS , Fuz.ZO },
          { Fuz.PB , Fuz.PM , Fuz.PS , Fuz.ZO , Fuz.NS , Fuz.ZO , Fuz.NM },
          { Fuz.PM , Fuz.PS , Fuz.PS , Fuz.NS , Fuz.NS , Fuz.NM , Fuz.NB },
          { Fuz.ZO , Fuz.PS , Fuz.NS , Fuz.NB , Fuz.NS , Fuz.PS , Fuz.ZO },
          { Fuz.NB , Fuz.NM , Fuz.NS , Fuz.NS , Fuz.PS , Fuz.PS , Fuz.PM },
          { Fuz.NM , Fuz.ZO , Fuz.NS , Fuz.ZO , Fuz.PS , Fuz.PM , Fuz.PB },
          { Fuz.ZO , Fuz.NS , Fuz.ZO , Fuz.PS , Fuz.PM , Fuz.PB , Fuz.PB },
        };
        Fuz[,] RuleI = new Fuz[7, 7]
        {
          { Fuz.NB , Fuz.NB , Fuz.NM , Fuz.NM , Fuz.NS , Fuz.ZO , Fuz.ZO },
          { Fuz.NB , Fuz.NB , Fuz.NM , Fuz.NS , Fuz.NS , Fuz.ZO , Fuz.ZO },
          { Fuz.NB , Fuz.NM , Fuz.NS , Fuz.NS , Fuz.ZO , Fuz.PS , Fuz.PS },
          { Fuz.NM , Fuz.NM , Fuz.NS , Fuz.ZO , Fuz.PS , Fuz.PM , Fuz.PM },
          { Fuz.NM , Fuz.NS , Fuz.ZO , Fuz.PS , Fuz.PS , Fuz.PM , Fuz.PB },
          { Fuz.ZO , Fuz.ZO , Fuz.PS , Fuz.PS , Fuz.PM , Fuz.PB , Fuz.PB },
          { Fuz.ZO , Fuz.ZO , Fuz.PS , Fuz.PM , Fuz.PM , Fuz.PB , Fuz.PB },
        };
        Fuz[,] RuleD = new Fuz[7, 7]
        {
          { Fuz.PB , Fuz.PM , Fuz.PS , Fuz.ZO , Fuz.NS , Fuz.NM , Fuz.NB },
          { Fuz.PM , Fuz.PS , Fuz.NB , Fuz.ZO , Fuz.NM , Fuz.NS , Fuz.NS },
          { Fuz.PM , Fuz.ZO , Fuz.ZO , Fuz.ZO , Fuz.ZO , Fuz.ZO , Fuz.ZO },
          { Fuz.PS , Fuz.ZO , Fuz.NS , Fuz.NM , Fuz.NS , Fuz.ZO , Fuz.PS },
          { Fuz.ZO , Fuz.ZO , Fuz.ZO , Fuz.ZO , Fuz.ZO , Fuz.ZO , Fuz.PM },
          { Fuz.NS , Fuz.NS , Fuz.NM , Fuz.ZO , Fuz.NB , Fuz.PS , Fuz.PM },
          { Fuz.NB , Fuz.NM , Fuz.NS , Fuz.ZO , Fuz.PS , Fuz.PM , Fuz.PB },
        };
        char [,]FuzzyP_Arg = new char[7,7];
        void Angular_FuzPID_Init()
        {
            Angular_FuzPID.Maximum = 300;
            Angular_FuzPID.Minimum = -300;

            Angular_FuzPID.Proportion = 10;
            Angular_FuzPID.MaxProportion = 25;
            Angular_FuzPID.MinProportion = 10;
            Angular_FuzPID.Proportion_K = 0.0f;

            Angular_FuzPID.Integral = 0;
            Angular_FuzPID.MaxIntegral = 5;
            Angular_FuzPID.MinIntegral = -5;
            Angular_FuzPID.Integral_K = 0.0f;

            Angular_FuzPID.Derivative = 10;
            Angular_FuzPID.MaxDerivative = 20;
            Angular_FuzPID.MinDerivative = 5;
            Angular_FuzPID.Derivative_K = 0.0f;

            Angular_FuzPID.E = 0;
            Angular_FuzPID.indexE = new byte[2] { 0, 0 };
            Angular_FuzPID.MsE = new float[2] { 0f, 0f };

            Angular_FuzPID.EC = 0;
            Angular_FuzPID.indexEC = new byte[2] { 0, 0 };
            Angular_FuzPID.MsEC = new float[2] { 0f, 0f };

            Angular_FuzPID.LastError = 0;
            Angular_FuzPID.PrevError = 0;
            Angular_FuzPID.SumError = 0;
            Angular_FuzPID.Output = 0;
        }
        void MassCenter_FuzPID_Init()
        {
            MassCenter_FuzPID.Maximum = 300;
            MassCenter_FuzPID.Minimum = -300;

            MassCenter_FuzPID.Proportion = 10;
            MassCenter_FuzPID.MaxProportion = 25;
            MassCenter_FuzPID.MinProportion = 10;
            MassCenter_FuzPID.Proportion_K = 0.0f;

            MassCenter_FuzPID.Integral = 0;
            MassCenter_FuzPID.MaxIntegral = 5;
            MassCenter_FuzPID.MinIntegral = -5;
            MassCenter_FuzPID.Integral_K = 0.0f;

            MassCenter_FuzPID.Derivative = 10;
            MassCenter_FuzPID.MaxDerivative = 20;
            MassCenter_FuzPID.MinDerivative = 5;
            MassCenter_FuzPID.Derivative_K = 0.0f;

            MassCenter_FuzPID.E = 0;
            MassCenter_FuzPID.indexE = new byte[2] { 0, 0 };
            MassCenter_FuzPID.MsE = new float[2] { 0f, 0f };

            MassCenter_FuzPID.EC = 0;
            MassCenter_FuzPID.indexEC = new byte[2] { 0, 0 };
            MassCenter_FuzPID.MsEC = new float[2] { 0f, 0f };

            MassCenter_FuzPID.LastError = 0;
            MassCenter_FuzPID.PrevError = 0;
            MassCenter_FuzPID.SumError = 0;
            MassCenter_FuzPID.Output = 0;
        }
        void LinearQuantization()  //模糊化，内部调用
        {
            float iError;
            float DeltaError;
            //横摆角速度PID计算
            iError = Expt_AnguarSpeed - Actual_AnguarSpeed;
            DeltaError = iError - Angular_FuzPID.LastError;

            Angular_FuzPID.E = 12.0f * iError / (Angular_FuzPID.Maximum - Angular_FuzPID.Minimum);
            Angular_FuzPID.EC = 60.0f * DeltaError / (Angular_FuzPID.Maximum - Angular_FuzPID.Minimum);
            if (Angular_FuzPID.E > 6)
                Angular_FuzPID.E = 6;
            else if (Angular_FuzPID.E < -6)
                Angular_FuzPID.E = -6;
            if (Angular_FuzPID.EC > 6)
                Angular_FuzPID.EC = 6;
            else if (Angular_FuzPID.EC < -6)
                Angular_FuzPID.EC = -6;
            //质心侧偏角PID计算
            iError = d_MassCenter;
            DeltaError = iError - MassCenter_FuzPID.LastError;

            MassCenter_FuzPID.E = 12.0f * iError / (MassCenter_FuzPID.Maximum - MassCenter_FuzPID.Minimum);
            MassCenter_FuzPID.EC = 60.0f * DeltaError / (MassCenter_FuzPID.Maximum - MassCenter_FuzPID.Minimum);
            if (MassCenter_FuzPID.E > 6)
                MassCenter_FuzPID.E = 6;
            else if (MassCenter_FuzPID.E < -6)
                MassCenter_FuzPID.E = -6;
            if (MassCenter_FuzPID.EC > 6)
                MassCenter_FuzPID.EC = 6;
            else if (MassCenter_FuzPID.EC < -6)
                MassCenter_FuzPID.EC = -6;
        }
        void CalMembership()
        {
            //横摆角速度隶属度计算
            if ((Angular_FuzPID.E >= (float)Fuz.NB) && (Angular_FuzPID.E < (float)Fuz.NM))
            {
                Angular_FuzPID.indexE[0] = 0;
                Angular_FuzPID.indexE[1] = 1;
                Angular_FuzPID.MsE[0] = -0.5f * Angular_FuzPID.E - 2.0f;
                Angular_FuzPID.MsE[1] = 0.5f * Angular_FuzPID.E + 3.0f;
            }
            else if ((Angular_FuzPID.E >= (float)Fuz.NM) && (Angular_FuzPID.E < (float)Fuz.NS))
            {
                Angular_FuzPID.indexE[0] = 1;
                Angular_FuzPID.indexE[1] = 2;
                Angular_FuzPID.MsE[0] = -0.5f * Angular_FuzPID.E - 1.0f;
                Angular_FuzPID.MsE[1] = 0.5f * Angular_FuzPID.E + 2.0f;
            }
            else if ((Angular_FuzPID.E >= (float)Fuz.NS) && (Angular_FuzPID.E < (float)Fuz.ZO))
            {
                Angular_FuzPID.indexE[0] = 2;
                Angular_FuzPID.indexE[1] = 3;
                Angular_FuzPID.MsE[0] = -0.5f * Angular_FuzPID.E;
                Angular_FuzPID.MsE[1] = 0.5f * Angular_FuzPID.E + 1.0f;
            }
            else if ((Angular_FuzPID.E >= (float)Fuz.ZO) && (Angular_FuzPID.E < (float)Fuz.PS))
            {
                Angular_FuzPID.indexE[0] = 3;
                Angular_FuzPID.indexE[1] = 4;
                Angular_FuzPID.MsE[0] = -0.5f * Angular_FuzPID.E + 1.0f;
                Angular_FuzPID.MsE[1] = 0.5f * Angular_FuzPID.E;
            }
            else if ((Angular_FuzPID.E >= (float)Fuz.PS) && (Angular_FuzPID.E < (float)Fuz.PM))
            {
                Angular_FuzPID.indexE[0] = 4;
                Angular_FuzPID.indexE[1] = 5;
                Angular_FuzPID.MsE[0] = -0.5f * Angular_FuzPID.E + 2.0f;
                Angular_FuzPID.MsE[1] = 0.5f * Angular_FuzPID.E - 1.0f;
            }
            else if ((Angular_FuzPID.E >= (float)Fuz.PM) && (Angular_FuzPID.E < (float)Fuz.PB))
            {
                Angular_FuzPID.indexE[0] = 5;
                Angular_FuzPID.indexE[1] = 6;
                Angular_FuzPID.MsE[0] = -0.5f * Angular_FuzPID.E + 3.0f;
                Angular_FuzPID.MsE[1] = 0.5f * Angular_FuzPID.E - 2.0f;
            }

            if ((Angular_FuzPID.EC >= (float)Fuz.NB) && (Angular_FuzPID.EC < (float)Fuz.NM))
            {
                Angular_FuzPID.indexEC[0] = 0;
                Angular_FuzPID.indexEC[1] = 1;
                Angular_FuzPID.MsEC[0] = -0.5f * Angular_FuzPID.EC - 2.0f;
                Angular_FuzPID.MsEC[1] = 0.5f * Angular_FuzPID.EC + 3.0f;
            }
            else if ((Angular_FuzPID.EC >= (float)Fuz.NM) && (Angular_FuzPID.EC < (float)Fuz.NS))
            {
                Angular_FuzPID.indexEC[0] = 1;
                Angular_FuzPID.indexEC[1] = 2;
                Angular_FuzPID.MsEC[0] = -0.5f * Angular_FuzPID.EC - 1.0f;
                Angular_FuzPID.MsEC[1] = 0.5f * Angular_FuzPID.EC + 2.0f;
            }
            else if ((Angular_FuzPID.EC >= (float)Fuz.NS) && (Angular_FuzPID.EC < (float)Fuz.ZO))
            {
                Angular_FuzPID.indexEC[0] = 2;
                Angular_FuzPID.indexEC[1] = 3;
                Angular_FuzPID.MsEC[0] = -0.5f * Angular_FuzPID.EC;
                Angular_FuzPID.MsEC[1] = 0.5f * Angular_FuzPID.EC + 1.0f;
            }
            else if ((Angular_FuzPID.EC >= (float)Fuz.ZO) && (Angular_FuzPID.EC < (float)Fuz.PS))
            {
                Angular_FuzPID.indexEC[0] = 3;
                Angular_FuzPID.indexEC[1] = 4;
                Angular_FuzPID.MsEC[0] = -0.5f * Angular_FuzPID.EC + 1.0f;
                Angular_FuzPID.MsEC[1] = 0.5f * Angular_FuzPID.EC;
            }
            else if ((Angular_FuzPID.EC >= (float)Fuz.PS) && (Angular_FuzPID.EC < (float)Fuz.PM))
            {
                Angular_FuzPID.indexEC[0] = 4;
                Angular_FuzPID.indexEC[1] = 5;
                Angular_FuzPID.MsEC[0] = -0.5f * Angular_FuzPID.EC + 2.0f;
                Angular_FuzPID.MsEC[1] = 0.5f * Angular_FuzPID.EC - 1.0f;
            }
            else if ((Angular_FuzPID.EC >= (float)Fuz.PM) && (Angular_FuzPID.EC < (float)Fuz.PB))
            {
                Angular_FuzPID.indexEC[0] = 5;
                Angular_FuzPID.indexEC[1] = 6;
                Angular_FuzPID.MsEC[0] = -0.5f * Angular_FuzPID.EC + 3.0f;
                Angular_FuzPID.MsEC[1] = 0.5f * Angular_FuzPID.EC - 2.0f;
            }
            //质心侧偏角隶属度计算
            if ((MassCenter_FuzPID.E >= (float)Fuz.NB) && (MassCenter_FuzPID.E < (float)Fuz.NM))
            {
                MassCenter_FuzPID.indexE[0] = 0;
                MassCenter_FuzPID.indexE[1] = 1;
                MassCenter_FuzPID.MsE[0] = -0.5f * MassCenter_FuzPID.E - 2.0f;
                MassCenter_FuzPID.MsE[1] = 0.5f * MassCenter_FuzPID.E + 3.0f;
            }
            else if ((MassCenter_FuzPID.E >= (float)Fuz.NM) && (MassCenter_FuzPID.E < (float)Fuz.NS))
            {
                MassCenter_FuzPID.indexE[0] = 1;
                MassCenter_FuzPID.indexE[1] = 2;
                MassCenter_FuzPID.MsE[0] = -0.5f * MassCenter_FuzPID.E - 1.0f;
                MassCenter_FuzPID.MsE[1] = 0.5f * MassCenter_FuzPID.E + 2.0f;
            }
            else if ((MassCenter_FuzPID.E >= (float)Fuz.NS) && (MassCenter_FuzPID.E < (float)Fuz.ZO))
            {
                MassCenter_FuzPID.indexE[0] = 2;
                MassCenter_FuzPID.indexE[1] = 3;
                MassCenter_FuzPID.MsE[0] = -0.5f * MassCenter_FuzPID.E;
                MassCenter_FuzPID.MsE[1] = 0.5f * MassCenter_FuzPID.E + 1.0f;
            }
            else if ((MassCenter_FuzPID.E >= (float)Fuz.ZO) && (MassCenter_FuzPID.E < (float)Fuz.PS))
            {
                MassCenter_FuzPID.indexE[0] = 3;
                MassCenter_FuzPID.indexE[1] = 4;
                MassCenter_FuzPID.MsE[0] = -0.5f * MassCenter_FuzPID.E + 1.0f;
                MassCenter_FuzPID.MsE[1] = 0.5f * MassCenter_FuzPID.E;
            }
            else if ((MassCenter_FuzPID.E >= (float)Fuz.PS) && (MassCenter_FuzPID.E < (float)Fuz.PM))
            {
                MassCenter_FuzPID.indexE[0] = 4;
                MassCenter_FuzPID.indexE[1] = 5;
                MassCenter_FuzPID.MsE[0] = -0.5f * MassCenter_FuzPID.E + 2.0f;
                MassCenter_FuzPID.MsE[1] = 0.5f * MassCenter_FuzPID.E - 1.0f;
            }
            else if ((MassCenter_FuzPID.E >= (float)Fuz.PM) && (MassCenter_FuzPID.E < (float)Fuz.PB))
            {
                MassCenter_FuzPID.indexE[0] = 5;
                MassCenter_FuzPID.indexE[1] = 6;
                MassCenter_FuzPID.MsE[0] = -0.5f * MassCenter_FuzPID.E + 3.0f;
                MassCenter_FuzPID.MsE[1] = 0.5f * MassCenter_FuzPID.E - 2.0f;
            }

            if ((MassCenter_FuzPID.EC >= (float)Fuz.NB) && (MassCenter_FuzPID.EC < (float)Fuz.NM))
            {
                MassCenter_FuzPID.indexEC[0] = 0;
                MassCenter_FuzPID.indexEC[1] = 1;
                MassCenter_FuzPID.MsEC[0] = -0.5f * MassCenter_FuzPID.EC - 2.0f;
                MassCenter_FuzPID.MsEC[1] = 0.5f * MassCenter_FuzPID.EC + 3.0f;
            }
            else if ((MassCenter_FuzPID.EC >= (float)Fuz.NM) && (MassCenter_FuzPID.EC < (float)Fuz.NS))
            {
                MassCenter_FuzPID.indexEC[0] = 1;
                MassCenter_FuzPID.indexEC[1] = 2;
                MassCenter_FuzPID.MsEC[0] = -0.5f * MassCenter_FuzPID.EC - 1.0f;
                MassCenter_FuzPID.MsEC[1] = 0.5f * MassCenter_FuzPID.EC + 2.0f;
            }
            else if ((MassCenter_FuzPID.EC >= (float)Fuz.NS) && (MassCenter_FuzPID.EC < (float)Fuz.ZO))
            {
                MassCenter_FuzPID.indexEC[0] = 2;
                MassCenter_FuzPID.indexEC[1] = 3;
                MassCenter_FuzPID.MsEC[0] = -0.5f * MassCenter_FuzPID.EC;
                MassCenter_FuzPID.MsEC[1] = 0.5f * MassCenter_FuzPID.EC + 1.0f;
            }
            else if ((MassCenter_FuzPID.EC >= (float)Fuz.ZO) && (MassCenter_FuzPID.EC < (float)Fuz.PS))
            {
                MassCenter_FuzPID.indexEC[0] = 3;
                MassCenter_FuzPID.indexEC[1] = 4;
                MassCenter_FuzPID.MsEC[0] = -0.5f * MassCenter_FuzPID.EC + 1.0f;
                MassCenter_FuzPID.MsEC[1] = 0.5f * MassCenter_FuzPID.EC;
            }
            else if ((MassCenter_FuzPID.EC >= (float)Fuz.PS) && (MassCenter_FuzPID.EC < (float)Fuz.PM))
            {
                MassCenter_FuzPID.indexEC[0] = 4;
                MassCenter_FuzPID.indexEC[1] = 5;
                MassCenter_FuzPID.MsEC[0] = -0.5f * MassCenter_FuzPID.EC + 2.0f;
                MassCenter_FuzPID.MsEC[1] = 0.5f * MassCenter_FuzPID.EC - 1.0f;
            }
            else if ((MassCenter_FuzPID.EC >= (float)Fuz.PM) && (MassCenter_FuzPID.EC < (float)Fuz.PB))
            {
                MassCenter_FuzPID.indexEC[0] = 5;
                MassCenter_FuzPID.indexEC[1] = 6;
                MassCenter_FuzPID.MsEC[0] = -0.5f * MassCenter_FuzPID.EC + 3.0f;
                MassCenter_FuzPID.MsEC[1] = 0.5f * MassCenter_FuzPID.EC - 2.0f;
            }
        }
        void FuzzyComputation()
        {
            float dP, dI, dD;

            LinearQuantization();

            CalMembership();
            //横摆角速度PID参数计算
            dP = Angular_FuzPID.MsE[0] * (Angular_FuzPID.MsEC[0] * (float)RuleP[Angular_FuzPID.indexE[0], Angular_FuzPID.indexEC[0]] + Angular_FuzPID.MsEC[1] * (float)RuleP[Angular_FuzPID.indexE[0], Angular_FuzPID.indexEC[1]])
               + Angular_FuzPID.MsE[1] * (Angular_FuzPID.MsEC[0] * (float)RuleP[Angular_FuzPID.indexE[1], Angular_FuzPID.indexEC[0]] + Angular_FuzPID.MsEC[1] * (float)RuleP[Angular_FuzPID.indexE[1], Angular_FuzPID.indexEC[1]]);
            dI = Angular_FuzPID.MsE[0] * (Angular_FuzPID.MsEC[0] * (float)RuleI[Angular_FuzPID.indexE[0], Angular_FuzPID.indexEC[0]] + Angular_FuzPID.MsEC[1] * (float)RuleI[Angular_FuzPID.indexE[0], Angular_FuzPID.indexEC[1]])
               + Angular_FuzPID.MsE[1] * (Angular_FuzPID.MsEC[0] * (float)RuleI[Angular_FuzPID.indexE[1], Angular_FuzPID.indexEC[0]] + Angular_FuzPID.MsEC[1] * (float)RuleI[Angular_FuzPID.indexE[1], Angular_FuzPID.indexEC[1]]);
            dD = Angular_FuzPID.MsE[0] * (Angular_FuzPID.MsEC[0] * (float)RuleD[Angular_FuzPID.indexE[0], Angular_FuzPID.indexEC[0]] + Angular_FuzPID.MsEC[1] * (float)RuleD[Angular_FuzPID.indexE[0], Angular_FuzPID.indexEC[1]])
               + Angular_FuzPID.MsE[1] * (Angular_FuzPID.MsEC[0] * (float)RuleD[Angular_FuzPID.indexE[1], Angular_FuzPID.indexEC[0]] + Angular_FuzPID.MsEC[1] * (float)RuleD[Angular_FuzPID.indexE[1], Angular_FuzPID.indexEC[1]]);

            if (Angular_FuzPID.Proportion <= Angular_FuzPID.MaxProportion && Angular_FuzPID.Proportion >= Angular_FuzPID.MinProportion)
            {
                Angular_FuzPID.Proportion += Angular_FuzPID.Proportion_K * dP;
                if (Angular_FuzPID.Proportion > Angular_FuzPID.MaxProportion)
                    Angular_FuzPID.Proportion = Angular_FuzPID.MaxProportion;
                else if (Angular_FuzPID.Proportion < Angular_FuzPID.MinProportion)
                    Angular_FuzPID.Proportion = Angular_FuzPID.MinProportion;
            }
            if (Angular_FuzPID.Integral <= Angular_FuzPID.MaxIntegral && Angular_FuzPID.Integral >= Angular_FuzPID.MinIntegral)
            {
                Angular_FuzPID.Integral += Angular_FuzPID.Integral_K * dI;
                if (Angular_FuzPID.Integral > Angular_FuzPID.MaxIntegral)
                    Angular_FuzPID.Integral = Angular_FuzPID.MaxIntegral;
                else if (Angular_FuzPID.Integral < Angular_FuzPID.MinIntegral)
                    Angular_FuzPID.Integral = Angular_FuzPID.MinIntegral;
            }
            if (Angular_FuzPID.Derivative <= Angular_FuzPID.MaxDerivative && Angular_FuzPID.Derivative >= Angular_FuzPID.MinDerivative)
            {
                Angular_FuzPID.Derivative += Angular_FuzPID.Derivative_K * dD;
                if (Angular_FuzPID.Derivative > Angular_FuzPID.MaxDerivative)
                    Angular_FuzPID.Derivative = Angular_FuzPID.MaxDerivative;
                else if (Angular_FuzPID.Derivative < Angular_FuzPID.MinDerivative)
                    Angular_FuzPID.Derivative = Angular_FuzPID.MinDerivative;
            }
            //质心侧偏角PID参数计算
            dP = MassCenter_FuzPID.MsE[0] * (MassCenter_FuzPID.MsEC[0] * (float)RuleP[MassCenter_FuzPID.indexE[0], MassCenter_FuzPID.indexEC[0]] + MassCenter_FuzPID.MsEC[1] * (float)RuleP[MassCenter_FuzPID.indexE[0], MassCenter_FuzPID.indexEC[1]])
               + MassCenter_FuzPID.MsE[1] * (MassCenter_FuzPID.MsEC[0] * (float)RuleP[MassCenter_FuzPID.indexE[1], MassCenter_FuzPID.indexEC[0]] + MassCenter_FuzPID.MsEC[1] * (float)RuleP[MassCenter_FuzPID.indexE[1], MassCenter_FuzPID.indexEC[1]]);
            dI = MassCenter_FuzPID.MsE[0] * (MassCenter_FuzPID.MsEC[0] * (float)RuleI[MassCenter_FuzPID.indexE[0], MassCenter_FuzPID.indexEC[0]] + MassCenter_FuzPID.MsEC[1] * (float)RuleI[MassCenter_FuzPID.indexE[0], MassCenter_FuzPID.indexEC[1]])
               + MassCenter_FuzPID.MsE[1] * (MassCenter_FuzPID.MsEC[0] * (float)RuleI[MassCenter_FuzPID.indexE[1], MassCenter_FuzPID.indexEC[0]] + MassCenter_FuzPID.MsEC[1] * (float)RuleI[MassCenter_FuzPID.indexE[1], MassCenter_FuzPID.indexEC[1]]);
            dD = MassCenter_FuzPID.MsE[0] * (MassCenter_FuzPID.MsEC[0] * (float)RuleD[MassCenter_FuzPID.indexE[0], MassCenter_FuzPID.indexEC[0]] + MassCenter_FuzPID.MsEC[1] * (float)RuleD[MassCenter_FuzPID.indexE[0], MassCenter_FuzPID.indexEC[1]])
               + MassCenter_FuzPID.MsE[1] * (MassCenter_FuzPID.MsEC[0] * (float)RuleD[MassCenter_FuzPID.indexE[1], MassCenter_FuzPID.indexEC[0]] + MassCenter_FuzPID.MsEC[1] * (float)RuleD[MassCenter_FuzPID.indexE[1], MassCenter_FuzPID.indexEC[1]]);

            if (MassCenter_FuzPID.Proportion <= MassCenter_FuzPID.MaxProportion && MassCenter_FuzPID.Proportion >= MassCenter_FuzPID.MinProportion)
            {
                MassCenter_FuzPID.Proportion += MassCenter_FuzPID.Proportion_K * dP;
                if (MassCenter_FuzPID.Proportion > MassCenter_FuzPID.MaxProportion)
                    MassCenter_FuzPID.Proportion = MassCenter_FuzPID.MaxProportion;
                else if (MassCenter_FuzPID.Proportion < MassCenter_FuzPID.MinProportion)
                    MassCenter_FuzPID.Proportion = MassCenter_FuzPID.MinProportion;
            }
            if (MassCenter_FuzPID.Integral <= MassCenter_FuzPID.MaxIntegral && MassCenter_FuzPID.Integral >= MassCenter_FuzPID.MinIntegral)
            {
                MassCenter_FuzPID.Integral += MassCenter_FuzPID.Integral_K * dI;
                if (MassCenter_FuzPID.Integral > MassCenter_FuzPID.MaxIntegral)
                    MassCenter_FuzPID.Integral = MassCenter_FuzPID.MaxIntegral;
                else if (MassCenter_FuzPID.Integral < MassCenter_FuzPID.MinIntegral)
                    MassCenter_FuzPID.Integral = MassCenter_FuzPID.MinIntegral;
            }
            if (MassCenter_FuzPID.Derivative <= MassCenter_FuzPID.MaxDerivative && MassCenter_FuzPID.Derivative >= MassCenter_FuzPID.MinDerivative)
            {
                MassCenter_FuzPID.Derivative += MassCenter_FuzPID.Derivative_K * dD;
                if (MassCenter_FuzPID.Derivative > MassCenter_FuzPID.MaxDerivative)
                    MassCenter_FuzPID.Derivative = MassCenter_FuzPID.MaxDerivative;
                else if (MassCenter_FuzPID.Derivative < MassCenter_FuzPID.MinDerivative)
                    MassCenter_FuzPID.Derivative = MassCenter_FuzPID.MinDerivative;
            }
        }
        void Loc_FuzzyPIDCal()  //位置模糊PIDi计算
        {
            float iError, dError;
            FuzzyComputation();
            //横摆角速度PID计算
            iError = Expt_AnguarSpeed - Actual_AnguarSpeed;
            Angular_FuzPID.SumError += iError;
            dError = iError - Angular_FuzPID.LastError;
            Angular_FuzPID.LastError = iError;
            Angular_FuzPID.Output = (long)(Angular_FuzPID.Proportion * iError
                                         + Angular_FuzPID.Integral * Angular_FuzPID.SumError
                                         + Angular_FuzPID.Derivative * dError);
            //质心侧偏角PID计算
            iError = d_MassCenter;
            MassCenter_FuzPID.SumError += iError;
            dError = iError - MassCenter_FuzPID.LastError;
            MassCenter_FuzPID.LastError = iError;
            MassCenter_FuzPID.Output = (long)(MassCenter_FuzPID.Proportion * iError
                                            + MassCenter_FuzPID.Integral * MassCenter_FuzPID.SumError
                                            + MassCenter_FuzPID.Derivative * dError);
        }
        void Fuzzy_Info()
        {
            for (byte i = 0; i < 7; i++)
            {
                for (byte j = 0; j < 7; j++)
                    FuzzyP_Arg[i, j] = ' ';
            }
            SetText_2("  dKp |负大|负中|负小|零零|正小|正中|正大|");
            SetText_2("|负大|  " + FuzzyP_Arg[0, 0] + RuleP[0, 0] + "  " + FuzzyP_Arg[0, 1] + RuleP[0, 1] + "  " + FuzzyP_Arg[0, 2] + RuleP[0, 2] + "  " + FuzzyP_Arg[0, 3] + RuleP[0, 3] + "  " + FuzzyP_Arg[0, 4] + RuleP[0, 4] + "  " + FuzzyP_Arg[0, 5] + RuleP[0, 5] + "  " + FuzzyP_Arg[0, 6] + RuleP[0, 6]);
            SetText_2("|负中|  " + FuzzyP_Arg[1, 0] + RuleP[0, 0] + "  " + FuzzyP_Arg[1, 1] + RuleP[0, 1] + "  " + FuzzyP_Arg[1, 2] + RuleP[0, 2] + "  " + FuzzyP_Arg[1, 3] + RuleP[0, 3] + "  " + FuzzyP_Arg[1, 4] + RuleP[1, 4] + "  " + FuzzyP_Arg[1, 5] + RuleP[0, 5] + "  " + FuzzyP_Arg[1, 6] + RuleP[0, 6]);
            SetText_2("|负小|  " + FuzzyP_Arg[2, 0] + RuleP[0, 0] + "  " + FuzzyP_Arg[2, 1] + RuleP[0, 1] + "  " + FuzzyP_Arg[2, 2] + RuleP[0, 2] + "  " + FuzzyP_Arg[2, 3] + RuleP[0, 3] + "  " + FuzzyP_Arg[2, 4] + RuleP[2, 4] + "  " + FuzzyP_Arg[2, 5] + RuleP[0, 5] + "  " + FuzzyP_Arg[2, 6] + RuleP[0, 6]);
            SetText_2("|零零|  " + FuzzyP_Arg[3, 0] + RuleP[0, 0] + "  " + FuzzyP_Arg[3, 1] + RuleP[0, 1] + "  " + FuzzyP_Arg[3, 2] + RuleP[0, 2] + "  " + FuzzyP_Arg[3, 3] + RuleP[0, 3] + "  " + FuzzyP_Arg[3, 4] + RuleP[3, 4] + "  " + FuzzyP_Arg[3, 5] + RuleP[0, 5] + "  " + FuzzyP_Arg[3, 6] + RuleP[0, 6]);
            SetText_2("|正小|  " + FuzzyP_Arg[4, 0] + RuleP[0, 0] + "  " + FuzzyP_Arg[4, 1] + RuleP[0, 1] + "  " + FuzzyP_Arg[4, 2] + RuleP[0, 2] + "  " + FuzzyP_Arg[4, 3] + RuleP[0, 3] + "  " + FuzzyP_Arg[4, 4] + RuleP[4, 4] + "  " + FuzzyP_Arg[4, 5] + RuleP[0, 5] + "  " + FuzzyP_Arg[4, 6] + RuleP[0, 6]);
            SetText_2("|正中|  " + FuzzyP_Arg[5, 0] + RuleP[0, 0] + "  " + FuzzyP_Arg[5, 1] + RuleP[0, 1] + "  " + FuzzyP_Arg[5, 2] + RuleP[0, 2] + "  " + FuzzyP_Arg[5, 3] + RuleP[0, 3] + "  " + FuzzyP_Arg[5, 4] + RuleP[5, 4] + "  " + FuzzyP_Arg[5, 5] + RuleP[0, 5] + "  " + FuzzyP_Arg[5, 6] + RuleP[0, 6]);
            SetText_2("|正大|  " + FuzzyP_Arg[6, 0] + RuleP[0, 0] + "  " + FuzzyP_Arg[6, 1] + RuleP[0, 1] + "  " + FuzzyP_Arg[6, 2] + RuleP[0, 2] + "  " + FuzzyP_Arg[6, 3] + RuleP[0, 3] + "  " + FuzzyP_Arg[6, 4] + RuleP[6, 4] + "  " + FuzzyP_Arg[6, 5] + RuleP[0, 5] + "  " + FuzzyP_Arg[6, 6] + RuleP[0, 6]);
        }
        #endregion
        /***************变量初始化***************/
        void ImageInit()
        {
            for (y = 0; y < 70; y++)
            {
                //将横向搜索数组清零
                L_black[y] = 185;
                R_black[y] = 0;
                //L_Start[y] = 185;
                //Tang_Line[y] = 0;
                //L_Trans[y] = 185;
                //R_Trans[y] = 0;
                LCenter[y] = 0;
            }
            EightRegion_Init(); // 八领域扫描初始化
        }
        /***************图像场处理***************/
        void ImageProcess()
        {
            ScanLine(); // 八领域扫描并判断赛道类型（环岛、车库、十字、普通直道、急/缓弯道）
            Exract_Edge(); // 提取每行的赛道边界，用来计算中线
            Road_Deal(); // 对不同的赛道类型做处理
            MidLine_NoOffset_Set(); // 设置中线
            SetInitSeedLine(); // 设置下一帧的种子位置
            Straight_Judge(); // 长直道 单独判断
            Road_SpcJudge(); // 坡道、会车 单独判断
            SetText_1("Road Type " + RoadType);
            setText用户自定义("RoadType = " + RoadType);
            SetText_1(" ");
            SetText_1("All Finished!");
        }
        /***************完整的一场数据处理***************/
        public void SignalProcess()
        {
            ImageInit();//变量初始化 
            ImageProcess();//图像场处理
        }

        #region 显示提示信息
        void SetText_1(object value)
        {
            mf.settext_1(value);
        }
        void SetText_2(object value)
        {
            mf.settext_2(value);
        }
        void setText用户自定义(object value)//显示自定义信息
        {
            if (MyNrf.Form1.VoiceString == "")
            {
                MyNrf.Form1.VoiceString += value.ToString();
            }
            else
            {
                MyNrf.Form1.VoiceString += "\r\n" + value.ToString();
            }
        }
        #endregion
    }
}

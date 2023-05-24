#include "gurobi_c++.h"
#include <algorithm>
#include <thread>
#include <mutex>
#include <vector>
#include <future>
using namespace std;
#define JOP  6
#define PROCESS {4,2,3,2,2,2 }
#define JOP1 {1,2,3,4 }
#define JOP2 {3,4 }
#define JOP3 {1,2,3 }
#define JOP4 {3,4 }
#define JOP5 {3,4 }
#define JOP6 {3,4 }
#define AGV  3
#define MACHINE 4
//机器加工工序的时间。
#define MACHINE_TIME {11, 23, 21, 24,23, 24, 14, 26,25, 26, 24, 15,26, 17, 24, 26,25, 26, 24, 15,26, 17, 24, 26,11, 23, 21, 24,23, 24, 14, 26,25, 26, 24, 15,25, 26, 24, 15,26, 17, 24, 26,25, 26, 24, 15,26, 17, 24, 26,25, 26, 24, 15,26, 17, 24, 26}
#define m1 {11, 23, 21, 24}
#define m2 {23, 24, 14, 26}
#define m3 {25, 26, 24, 15}
#define m4 {26, 17, 24, 26}
#define TIME 500
//互斥锁
std::mutex mtx;

//机器的时间。
int
main(int   argc,
     char *argv[])
{
  try {
    // GRBEnv env = GRBEnv(true);
    //  env.set("LogFile", "1.log");
    //  env.start();
    // GRBModel model = GRBModel(env);
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
//选取工件的3维变量 工件在AGV到的执行位置
GRBVar*** Tisx=new GRBVar**[JOP];
for (int i = 0; i < JOP; i++)
{
     Tisx[i]=new GRBVar*[AGV];
     for (int s = 0; s < AGV; s++)
     {    
          Tisx[i][s]=new GRBVar[JOP];
          for (int x = 0; x < JOP; x++)
          {
            //GRB_INTEGER  GRB_BINARY
               Tisx[i][s][x]=model.addVar(0.0, 1.0, 0.0, GRB_INTEGER ,"Jop_"+ to_string(i) +"agv_"+ to_string(s)+"location_"+ to_string(x)  ); 
          }
          
     }
     
}
//添加约束
/*
1.将工件分配给个AGV。只有每个工件只能占用AGV当前的唯一位置。
*/
GRBLinExpr expr = 0;
for (int i = 0; i < JOP; i++) {
    for (int j = 0; j < AGV; j++) {
        for (int k = 0; k <JOP ; k++) {
            expr += Tisx[i][j][k];
        }
    }
}
model.addConstr(expr == JOP);
//一个工件只能在一个位置 对每一个工件加连续的变量约束
for (int i = 0; i < JOP; i++)
{
     GRBLinExpr expr1 = 0;
      for (int j = 0; j < AGV; j++) {
        for (int x = 0; x <JOP ; x++) {
            expr1 += Tisx[i][j][x];
        }
    }
    model.addConstr(expr1 == 1);
}
//一个工件只能在AGV一个位置上，不能出现多个工件在一个AGV相同的位置
for (int x = 0; x <JOP ; x++) {
 for (int j = 0; j < AGV; j++) {
  GRBLinExpr expr3 = 0;
  for (int i = 0; i < JOP; i++){
     expr3 += Tisx[i][j][x];
  }
  model.addConstr(expr3 <= 1);
 }
}
//变量对应的时间取值 machine_time= Pijk
int*** machin_time=new int**[JOP]; 
int p[]=MACHINE_TIME;
int s=0;
for (int i = 0; i < JOP; i++)
{
    int pro_time[]=PROCESS;
    machin_time[i]=new int*[pro_time[i]];
    for (int j = 0; j < pro_time[i]; j++)
    {
         machin_time[i][j]=new int[MACHINE];
          for (int k = 0; k < MACHINE; k++) {
             machin_time[i][j][k] =p[s++]; 
        }
    }
    
}
//输出结果Pijk
// for (int i = 0; i < JOP; i++)
// {
//     int pro_time[]=PROCESS;
//     for (int j = 0; j < pro_time[i]; j++)
//     {
         
//           for (int k = 0; k < MACHINE; k++) {
//             cout<< machin_time[i][j][k]<<" "; 
//         }
//         cout<<endl;
//     }
// }

GRBVar*** var=new GRBVar**[JOP];
//定义一个3维的变量  这是关于AGV 工序的变量  
for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    var[i] = new GRBVar*[process[i]];
    for (int j = 0; j <process[i]; j++) {
        var[i][j] = new GRBVar[MACHINE];
        for (int k = 0; k < MACHINE; k++) {
            string machine;
            //GRB_INTEGER GRB_BINARY
            var[i][j][k] =  model.addVar(0.0, 1.0, 0.0,GRB_INTEGER ,"jop_"+to_string(i)+"gongxu_"+to_string(j)+"machine_"+to_string(k)); // 添加二进制变量
        }
    }
}
//工件工序选择机器的约束
/*
3.每一个工件的工序只能在4个机器中选择一个。
*/
for (int  i = 0; i < JOP; i++)
{
    int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
        GRBLinExpr expr2 = 0;
        for (int k = 0; k < MACHINE; k++) {
            expr2+=var[i][j][k];
        }
         model.addConstr(expr2 == 1);
    }
}
//每一个工件的工序都必须选择机器
GRBLinExpr expr3 = 0;
for (int  i = 0; i < JOP; i++)
{
    int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
        
        for (int k = 0; k < MACHINE; k++) {
            expr3+=var[i][j][k];
        }
        
    }
}
//15 = 每个工件工序相加之和
 model.addConstr(expr3 == 15);
int machine_c[4]={0};
//GRBLinExpr = GRBQuadExpr
GRBVar agv_c[AGV];
for (int i = 0; i < AGV; i++)
{
    agv_c[i]=model.addVar(0.0, GRB_MAXINT, 0.0,GRB_INTEGER );
}

GRBVar mach_c[MACHINE];
for (int i = 0; i < MACHINE; i++)
{
    mach_c[i]=model.addVar(0.0, GRB_MAXINT, 0.0,GRB_INTEGER );
}
//各个工序的开始时间。
GRBVar** SPij =new GRBVar*[JOP];
for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    SPij[i]=new GRBVar[process[i]];
    for (int j = 0; j <process[i]; j++) {
        SPij[i][j] = model.addVar(0.0, GRB_MAXINT, 0.0,GRB_INTEGER ,"jop_state"+to_string(i)+"gongxu_"+to_string(j));
       
    }
}
//各个工序的结束时间。
GRBVar** EPij =new GRBVar*[JOP];
for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    EPij[i]=new GRBVar[process[i]];
    for (int j = 0; j <process[i]; j++) {
       EPij[i][j] =model.addVar(0.0, GRB_MAXINT, 0.0,GRB_INTEGER ,"jop_end"+to_string(i)+"gongxu_"+to_string(j));
       
    }
    
}

for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
       GRBQuadExpr asd1=0;        
        for (int k = 0; k < MACHINE; k++)
        {
            asd1+=var[i][j][k]*machin_time[i][j][k];
        }
        
        model.addConstr(EPij[i][j] >= SPij[i][j]+asd1);
    }
}
//定义初始的 i0的时间
for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
       GRBQuadExpr asd1=0;        
        for (int k = 0; k < MACHINE; k++)
        {
            asd1+=var[i][j][k]*machin_time[i][j][k];
        }
        
        model.addConstr(EPij[i][j] >= SPij[i][j]+asd1);
    }
}
GRBVar agv[3];
for (int i = 0; i < AGV; i++)
{
    GRBVar agv[i]=model.addVar(0.0, GRB_MAXINT, 0.0,GRB_INTEGER ,"AGV_STATE_TIME"+to_string(i));
}
       
for (int s = 0; s < AGV; s++) {  
    for (int x = 0; x <JOP ; x++) {
        GRBQuadExpr asd3[3];
        for (int i = 0; i < JOP; i++){
            

    }
 }
}
GRBQuadExpr obj1=EPij[0][0];
for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
        int abj1=obj1.getValue();
        int abj2=EPij[i][j].get(GRB_DoubleAttr_X);
       if (abj1<=abj2)
       {
            obj1=EPij[i][j];
       }
       
    }
}
model.setObjective( obj1, GRB_MINIMIZE);
model.optimize();


// for (int i = 0; i < JOP; i++)
// {
//      Tisx[i]=new GRBVar*[AGV];
//      for (int s = 0; s < AGV; s++)
//      {    
//           Tisx[i][s]=new GRBVar[JOP];
//           for (int x = 0; x < JOP; x++)
//           {
//               //GRB_DoubleAttr_ObjVal
//         cout << Tisx[i][s][x].get(GRB_DoubleAttr_ObjVal) << endl;
//           }
          
//      }
// }
for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
        for (int k = 0; k < MACHINE; k++) {
            if (var[i][j][k].get(GRB_DoubleAttr_X)==1.0)
            {
                
                cout<<"工件 "<<i<<"工序"<<j<<"机器"<<k<<endl;
            }
            

        }
    }
}
for (int i = 0; i < JOP; i++)
{
     for (int s = 0; s < AGV; s++)
     {    
          for (int x = 0; x < JOP; x++)
          {
            // cout<<Tisx[i][s][x].get(GRB_DoubleAttr_X);
            if( Tisx[i][s][x].get(GRB_DoubleAttr_X)==1.0)
               cout<<"AGV "<<s<<"运输工件"<<i<<"位置"<<x<<endl;
          }
        //   cout<<endl;
     }
     
}

//     cout << x.get(GRB_StringAttr_VarName) << " "
//          << x.get(GRB_DoubleAttr_X) << endl;

    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//  cout << Tisx[0][0][0].get(GRB_DoubleAttr_ObjVal) << endl;
// std::cout << "Optimal objective value: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

//释放内存
for (int i = 0; i < JOP; ++i) {
    for (int j = 0; j < AGV; ++j) {
        delete[] Tisx[i][j];
    }
    delete[] Tisx[i];
}
delete[] Tisx;
for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
         delete[] var[i][j];
    }
    delete[] var[i];
}
delete[] var;
for (int i = 0; i < JOP; i++)
{
    int pro_time[]=PROCESS;
    for (int j = 0; j < pro_time[i]; j++)
    {
          delete[] machin_time[i][j];
    }
     delete[] machin_time[i];
}

  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }
  return 0;
  
}

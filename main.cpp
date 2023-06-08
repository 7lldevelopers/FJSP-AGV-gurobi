#include "gurobi_c++.h"
#include <algorithm>
#include <thread>
#include <mutex>
#include <vector>
#include <future>
using namespace std;
#define JOP  6
//#define JOP  4
#define PROCESS {4,2,3,2,4,2}
//#define PROCESS {4,2,3,2}
#define JOP1 {1,2,3,4 }
#define JOP2 {3,4 }
#define JOP3 {1,2,3 }
#define JOP4 {3,4 }
#define JOP5 {1,2,3,4  }
#define JOP6 {1,4 }
//2
#define AGV  3
#define MACHINE 4
//机器加工工序的时间。
#define MACHINE_TIME {6, 7, 4, 3,3, 5, 6, 4,3, 2, 1, 4,3, 3, 2, 5,3, 2, 1, 4,3, 3, 2, 5,6, 7, 4, 3,3, 5, 6, 4,3, 2, 1, 4,3, 2, 1, 4,3, 3, 2, 5,6, 7, 4, 3,3, 5, 6, 4,3, 2, 1, 4,3, 3, 2, 5,6, 7, 4, 3,3, 3, 2, 5}
//#define MACHINE_TIME {6, 7, 4, 3,3, 5, 6, 4,3, 2, 1, 4,3, 3, 2, 5,3, 2, 1, 4,3, 3, 2, 5,6, 7, 4, 3,3, 5, 6, 4,3, 2, 1, 4,3, 2, 1, 4,3, 3, 2, 5}
#define m1 {6, 7, 4, 3}
#define m2 {3, 5, 6, 4}
#define m3 {3, 2, 9, 4}
#define m4 {3, 3, 2, 5}
#define gongjian 17
#define shang 5
#define xia 3
#define TIME 500
//每一次优化结果输出
class MyCallback : public GRBCallback {
public:
    void callback() {
        if (where == GRB_CB_MIP) {
            //double obj = getDoubleInfo(GRB_CB_MIP_OBJBST);
             double obj = getDoubleInfo(GRB_CB_MIP_OBJBST);
            cout << "Optimal solution found with objective value: " << obj << endl;
            for (int i = 0; i < JOP; i++) {
                int process[]=PROCESS;
                for (int j = 0; j <process[i]; j++) {

                 //    cout<< "工件_"<<i<<j<<"_的开始时间"<<getSolution( var[i][j][0])<<endl;
                    // cout<< "工件_"<<i<<j<<"_的结束时间"<<getSolution(EPij)<<endl;
                }
           // cout<<endl;
            }
            
        }
    }
};


//选取最大的值
GRBVar max_(GRBVar a,GRBVar b){
return a.get(GRB_DoubleAttr_X)>=b.get(GRB_DoubleAttr_X)? a:b ;
}
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
    env.set(GRB_IntParam_NumericFocus, 3);
     // env.set("LogFile", "mip1.log");
    //  env.start();
   //env.set(GRB_DoubleParam_TimeLimit, 60.0);
    GRBModel model = GRBModel(env);
    //选取工件的3维变量 工件在AGV到的执行位置
/*
********************************************************************************************************************************
*/
//变量对应的时间取值 machine_time= Pijk
int*** machin_time=new int**[JOP]; 
int p[]=MACHINE_TIME;
int s=0;
int mahcne_all=0;
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
//定义一个3维的变量  这是关于AGV 工序的变量  
GRBVar*** var=new GRBVar**[JOP];
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
 model.addConstr(expr3 == gongjian);//17
 // model.addConstr(expr3 == 11);

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

/*
********************************************************************************************************************************
*/
//***********************************FJSP-model
//表示 工件ij 比  工件ip jp提前加工。 
GRBVar**** y =new GRBVar***[JOP];
for (int  i = 0; i < JOP; i++)
{
    int process[]=PROCESS;
    y[i]=new GRBVar**[process[i]];
    for (int  j = 0; j < process[i]; j++)
    {
        y[i][j]=new GRBVar*[JOP];
        for (int ip = 0; ip < JOP; ip++)
        {
             int process[]=PROCESS;
             y[i][j][ip]=new GRBVar[process[ip]];
             for (int jp = 0; jp < process[ip]; jp++)
             {
                y[i][j][ip][jp]=model.addVar(0.0, 1.0, 0.0,GRB_INTEGER );
             }
             
        }
        
    }
    
}
//GRBLinExpr = GRBQuadExpr
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
//这里添加工序的前一个工序？？？？
//vs表示当前工件前一个加工的工件是谁
GRBVar** VS =new GRBVar*[JOP];
for (int i = 0; i < JOP; i++) {    
    VS[i]=new GRBVar[JOP];
    for (int j = 0; j <JOP; j++) {
       VS[i][j] =model.addVar(0.0, 1.0, 0.0,GRB_INTEGER );
       
    }
    
}
//
for (int i = 0; i < JOP; i++) {    
    GRBLinExpr TES=0;
    for (int j = 0; j <JOP; j++) {
       TES+=VS[i][j];
    }
    model.addConstr(TES <=1);
    
}
for (int i = 0; i < JOP; i++) {    
    GRBLinExpr TES=0;
    for (int j = 0; j <JOP; j++) {
       TES+=VS[j][i];
    }
    model.addConstr(TES <=1);
    
}
for (int i = 0; i < JOP; i++) {    
    for (int j = 0; j <JOP; j++) {
       model.addConstr(VS[i][j] + VS[j][i]<=1 );
    }
    
}
GRBLinExpr TES1=0;
for (int i = 0; i < JOP; i++) {    
    
    for (int j = 0; j <JOP; j++) {
       TES1+=VS[i][j];
    }
    
}
model.addConstr(TES1 >=xia);//4
model.addConstr(TES1 <=shang);//5
//model.addConstr(TES1 <=5);
int V = 100000; 
for (int i = 0; i < JOP; i++) {
GRBQuadExpr asdd=0;
for (int j = 0; j <JOP; j++) {
    int process[]=PROCESS;
    //这是前一个工件的完成时间
    asdd+=VS[i][j]*EPij[j][process[j]-1];
}
 model.addQConstr(SPij[i][0] >= asdd);
 //
        GRBQuadExpr asd7=0;        
        for (int k = 0; k < MACHINE; k++)
        {
            asd7+=var[i][0][k]*machin_time[i][0][k];
          
        }
        
        model.addConstr(EPij[i][0] >= SPij[i][0]+asd7);
//当前工序选择机器的完成时间？？？
      for (int ip = 0; ip < JOP; ip++)
        {
             if (i!=ip){
                int process[]=PROCESS;
                 for (int jp = 0; jp <process[ip]; jp++) 
                 {
                        for (int ik = 0; ik < MACHINE; ik++)
                        {
                             model.addConstr(SPij[i][0]>=EPij[ip][jp]-V*(3-y[i][0][ip][jp]-var[i][0][ik]-var[ip][jp][ik]));
                            model.addConstr(SPij[ip][jp]>=EPij[i][0]-V*(2+y[i][0][ip][jp]-var[i][0][ik]-var[ip][jp][ik]));
                   
                        }
                 }

             }
             
        }
    
}
//当前工件工序选择机器的结束时间：
//约束条件 EP SP
for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
       GRBQuadExpr asd1=0;        
        for (int k = 0; k < MACHINE; k++)
        {
            asd1+=var[i][j][k]*machin_time[i][j][k];
          
        }
        
        model.addConstr(EPij[i][j] >= SPij[i][j]+asd1);
        
        
        if (j!=0)
        { 
            //上一个工序结束的时间
            model.addConstr(SPij[i][j] >= EPij[i][j-1]);
            //**********************************************************************************************************
            //当前工序选择的机器最后结束的时间 
            
            for (int ip = 0; ip < JOP; ip++)
            {
                if (i!=ip)
                 {
                  int process[]=PROCESS;
                 for (int jp = 0; jp <process[ip]; jp++) 
                 {
                        for (int ik = 0; ik < MACHINE; ik++)
                         {
                            model.addConstr(SPij[i][j]>=EPij[ip][jp]-V*(3-y[i][j][ip][jp]-var[i][j][ik]-var[ip][jp][ik]));
                             model.addConstr(SPij[ip][jp]>=EPij[i][j]-V*(2+y[i][j][ip][jp]-var[i][j][ik]-var[ip][jp][ik]));
                         }
                  }
                 }
                
             }


            //>0
            model.addConstr(SPij[i][j] >=0);
        }

    
    }
        
}

//其中最大的时间最小化
GRBQuadExpr  OBJ=model.addVar(0.0, GRB_MAXINT, 0.0,GRB_INTEGER );
GRBQuadExpr obj1=0;

for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
       model.addConstr(OBJ >= EPij[i][j]);
    }
}
model.setObjective( OBJ, GRB_MINIMIZE);
// MyCallback cb;
// model.setCallback(&cb);
model.optimize();
for (int i = 0; i < JOP; i++)
{
    for (int j = 0; j < JOP; j++)
    {
         if (VS[i][j].get(GRB_DoubleAttr_X)==1.0){

            cout<<"工件"<<i<<"在"<<j<<"后面"<<endl;
         }
    }
    
}


for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
        cout<< "工件_"<<i<<j<<"_的开始时间"<<SPij[i][j].get(GRB_DoubleAttr_X)<<endl;
       cout<< "工件_"<<i<<j<<"_的结束时间"<<EPij[i][j].get(GRB_DoubleAttr_X)<<endl;
    }
    cout<<endl;
}

for (int i = 0; i < JOP; i++) {
     int process[]=PROCESS;
    for (int j = 0; j <process[i]; j++) {
        for (int k = 0; k < MACHINE; k++) {
            if (var[i][j][k].get(GRB_DoubleAttr_X)==1.0)
            {
                
                cout<<"工件 "<<i<<"工序"<<j<<"选取机器"<<k<<endl;
            }
            

        }
    }
}
for (int  i = 0; i < JOP; i++)
{
    int process[]=PROCESS;
    
    for (int  j = 0; j < process[i]; j++)
    {
       
        for (int ip = 0; ip < JOP; ip++)
        {
             int process[]=PROCESS;
            
             for (int jp = 0; jp < process[ip]; jp++)
             {
                
                if (y[i][j][ip][jp].get(GRB_DoubleAttr_X)==1.0)
                {
                
                cout<<"工件 "<<i<<"工序"<<j<<"  "<<"工件 "<<ip<<"工序"<<jp<<endl;
                 }
             }
             
        }
        
    }
    
}
    //输出目标值：
    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//释放内存
for (int i = 0; i < JOP; i++) {
    delete[] VS[i];
}
delete[] VS;
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
   int process[]=PROCESS; 
for (int j = 0; j < process[i]; j++)
{
for (int ip = 0; ip < JOP; ip++)
{
delete[] y[i][j][ip];
}
delete[] y[i][j];
}
delete[] y[i];
}
delete[] y;
for (int i = 0; i < JOP; i++)
{
    int pro_time[]=PROCESS;
    for (int j = 0; j < pro_time[i]; j++)
    {
          delete[] machin_time[i][j];
    }
     delete[] machin_time[i];
}
delete[] machin_time;
  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }
  return 0;
 
}


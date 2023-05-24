#include <iostream>
#include <vector>
#include "gurobi_c++.h"

using namespace std;


int main() {
    try {
        // 创建Gurobi环境
        GRBEnv env = GRBEnv();

        // 创建Gurobi模型
        GRBModel model = GRBModel(env);

        // 定义变量
        vector<vector<GRBVar>> x; // x[i][j]表示第i个任务在第j个时间段是否被分配
        vector<vector<GRBVar>> y; // y[i][j]表示第i个AGV在第j个时间段是否被分配
        vector<vector<vector<GRBVar>>> z; // z[i][j][k]表示第i个任务在第j个时间段是否被第k个AGV执行

        // 初始化变量
        int n = 10; // 任务数量
        int m = 5; // AGV数量
        int T = 20; // 时间段数量
        x.resize(n);
        y.resize(m);
        z.resize(n);
        for (int i = 0; i < n; i++) {
            x[i].resize(T);
            z[i].resize(T);
            for (int j = 0; j < T; j++) {
                x[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + to_string(i) + "_" + to_string(j));
                z[i][j].resize(m);
                for (int k = 0; k < m; k++) {
                    z[i][j][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
                }
            }
        }
        for (int i = 0; i < m; i++) {
            y[i].resize(T);
            for (int j = 0; j < T; j++) {
                y[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y_" + to_string(i) + "_" + to_string(j));
            }
        }

        // 添加约束
        for (int i = 0; i < n; i++) {
            GRBLinExpr expr = 0;
            for (int j = 0; j < T; j++) {
                expr += x[i][j];
            }
            model.addConstr(expr == 1, "task_" + to_string(i) + "_once");
        }
        for (int j = 0; j < T; j++) {
            for (int k = 0; k < m; k++) {
                GRBLinExpr expr = 0;
                for (int i = 0; i < n; i++) {
                    expr += z[i][j][k];
                }
                model.addConstr(expr <= y[k][j], "agv_" + to_string(k) + "_at_" + to_string(j));
            }
        }
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < T; j++) {
                GRBLinExpr expr = 0;
                for (int k = 0; k < m; k++) {
                    expr += z[i][j][k];
                }
                model.addConstr(expr == x[i][j], "task_" + to_string(i) + "_at_" + to_string(j));
            }
        }
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < T; j++) {
                for (int k = 0; k < m; k++) {
                    model.addConstr(z[i][j][k] <= y[k][j], "task_" + to_string(i) + "_at_" + to_string(j) + "_by_" + to_string(k));
                }
            }
        }

        // 定义目标函数
        GRBLinExpr obj = 0;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < T; j++) {
                for (int k = 0; k < m; k++) {
                    obj += z[i][j][k];
                }
            }
        }
        model.setObjective(obj, GRB_MAXIMIZE);

        // 求解模型
        model.optimize();

        // 输出结果
        cout << "Optimal objective: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < T; j++) {
                for (int k = 0; k < m; k++) {
                    if (z[i][j][k].get(GRB_DoubleAttr_X) > 0.5) {
                        cout << "Task " << i << " at time " << j << " by AGV " << k << endl;
                    }
                }
            }
        }
    } catch (GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Exception during optimization" << endl;
    }

    return 0;
}
// void machine_time(int* machine_c,int i,int time,int& agv_time) {
//     mtx.lock(); // 获取锁
//     if (machine_c[i]<=agv_time)
//     {
//         machine_c[i]=agv_time+time;
//         agv_time=agv_time+time;
        
//     }else{
//          machine_c[i]= machine_c[i]+time;
//          agv_time=machine_c[i]+time;
         
//     }
//     mtx.unlock(); // 释放锁
// }

// void thread_funcd(int s, GRBVar*** var, GRBVar*** Tisx,int*** machin_time,int* machine_c,int agv) {
//     int agv1=0;
//     //计算每一个AGV的目标值
//     for (int x = 0; x < JOP; x++)
//     {
//         for (int i = 0; i < JOP; i++)
//         {
//            if(Tisx[i][s][x].get(GRB_DoubleAttr_X)==1.0){
//                 int process1[]=PROCESS;
//                 for (int j = 0; j < process1[i]; j++)
//                 {
//                     for (int k = 0; k < MACHINE; i++)
//                     {
//                         if(var[i][j][k].get(GRB_DoubleAttr_X)==1.0){
//                             machine_time(machine_c,k,machin_time[i][j][k],agv);
//                         }
//                     }
                    
//                 }
                
//            }

//         }

//     }
   
// }
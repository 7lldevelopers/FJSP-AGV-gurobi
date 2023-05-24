#include <iostream>
#include <vector>
#include "gurobi_c++.h"

using namespace std;

const int n=4;
const int m =4;
const int p=4;
int main(int argc, char *argv[]) {
    try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);

        // 定义变量
        vector<vector<vector<GRBVar>>> x;
        for (int i = 0; i < n; i++) {
            vector<vector<GRBVar>> row;
            for (int j = 0; j < m; j++) {
                vector<GRBVar> col;
                for (int k = 0; k < p; k++) {
                    GRBVar var = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
                    col.push_back(var);
                }
                row.push_back(col);
            }
            x.push_back(row);
        }

        // 添加约束
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                GRBLinExpr expr = 0;
                for (int k = 0; k < p; k++) {
                    expr += x[i][j][k];
                }
                model.addConstr(expr == 1);
            }
        }

        // 添加目标函数
        GRBLinExpr obj = 0;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                for (int k = 0; k < p; k++) {
                    obj += x[i][j][k] * c[i][j][k];
                }
            }
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // 求解模型
        model.optimize();

        // 输出结果
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                for (int k = 0; k < p; k++) {
                    if (x[i][j][k].get(GRB_DoubleAttr_X) > 0.5) {
                        cout << "Job " << i << " on Machine " << j << " at Time " << k << endl;
                    }
                }
            }
        }

    } catch (GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during optimization" << endl;
    }

    return 0;
}


/* Copyright 2023, Gurobi Optimization, LLC */

/* This example formulates and solves the following simple QCP model:

     maximize    x
     subject to  x + y + z = 1
                 x^2 + y^2 <= z^2 (second-order cone)
                 x^2 <= yz        (rotated second-order cone)
                 x, y, z non-negative
*/

#include "gurobi_c++.h"
using namespace std;

int
main(int   argc,
     char *argv[])
{
  try {
//GRBEnv是Gurobi Optimizer的一个类，用于管理Gurobi环境。它提供了一种方法来设置Gurobi参数，创建和销毁Gurobi模型等。
//在使用Gurobi时，每个应用程序只需要创建一个GRBEnv对象。该对象将作为参数传递给所有其他Gurobi对象，例如GRBModel和GRBVar。
    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);

// Create variables
// model.addVar()是中Gurobi optimizer库的一个函数，用于向线性规划模型添加新的变量。
// 方法：    GRBVar addVar(double lb, double ub, double obj, char vtype,std::string vname="");
// 参数：   lb: 变量的下界（lower bound），默认值为 0。
//         ub: 变量的上界（upper bound），默认值为正无穷大。GRB_INFINITY=无限大。
//         obj: 变量的目标系数（objective coefficient），即该变量在目标函数中的系数，默认值为 0。
//         vtype: 变量的类型，可选值为 'C'（连续变量）和 'B'（二进制变量），默认值为 'C'。  GRB_INFINITY="c"
//         name: 变量的名称，可选参数。

    GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
    GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
    GRBVar z = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z");
  GRBVar t = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t");

    // Set objective
// GRBLinExp 便是为线性表达式 设置目标值： obj
// 它可以用于构建线性规划、混合整数线性规划等优化问题的目标函数和约束条件。
// GRBLinExpr的实例可以由常数、变量和常数与变量的乘积组成，支持常见的加、减、乘运算，并且可以使用各种运算符来组合不同的GRBLinExpr实例。
// 例如：GRBLinExpr expr = 3 * x + 4 * y;
    GRBLinExpr obj = x;
// GRB_MAXIMIZE =-1 最大值 设置目标的最大值
    model.setObjective(obj, GRB_MAXIMIZE);

    // Add linear constraint: x + y + z = 1
// 用于向数学优化模型对象添加限制条件(constraint)的方法。
//model.addConstr()用于添加一条线性约束，而model.addQConstr()用于添加一条二次约束。

    model.addConstr(x + y + z == 1, "c0");

    // Add second-order cone: x^2 + y^2 <= z^2

    model.addQConstr(x*x + y*y <= z*z, "qc0");

    // Add rotated cone: x^2 <= yz

    model.addQConstr(x*x <= y*z, "qc1");
    model.addConstr(x+y+t==2,"c1");

    // Optimize model
//使用optimize()方法来执行求解器以求解优化问题
    model.optimize();
//它返回一个 double 类型的值，表示相应变量的当前取值。在调用该函数之前，必须确保模型已经被求解，并且变量具有可行解。如果变量没有定义初始值或未定义边界，则该函数可能会返回未定义的值。
    cout << x.get(GRB_StringAttr_VarName) << " "
         << x.get(GRB_DoubleAttr_X) << endl;
    cout << y.get(GRB_StringAttr_VarName) << " "
         << y.get(GRB_DoubleAttr_X) << endl;
    cout << z.get(GRB_StringAttr_VarName) << " "
         << z.get(GRB_DoubleAttr_X) << endl;
cout << t.get(GRB_StringAttr_VarName) << " "
         << t.get(GRB_DoubleAttr_X) << endl;

    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }

  return 0;
}


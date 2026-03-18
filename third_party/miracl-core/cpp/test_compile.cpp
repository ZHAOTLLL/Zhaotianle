// 简单的编译测试程序
#include "core.h"
#include "ecp_BLS12381.h"
#include "big_B384_58.h"
#include <iostream>

using namespace core;
using namespace BLS12381;
using namespace B384_58;

int main() {
    std::cout << "测试 Miracl Core BLS12381 编译..." << std::endl;

    // 测试基本类型
    BIG a, b;
    BIG_zero(a);
    BIG_one(b);

    ECP P;
    ECP_generator(&P);

    std::cout << "BLS12381 初始化成功！" << std::endl;
    std::cout << "库编译正常，可以正常使用。" << std::endl;

    return 0;
}


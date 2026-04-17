#include <iostream>
#include <random>
#include <vector>

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_int(0, 8);
    std::vector<int> ex;
    for(int i = 0; i < 10; i ++) {
        // 生成并输出一个[0,8]的随机整数
        int x   = dis_int(gen);
        int f   = dis_int(gen);
        int len = dis_int(gen) * 1323 % 48;
        if(x % 4 < 2) {
            std::cout << "CRE " << f << " " << len + 1 << "\n";
            ex.push_back(f);
        } else if(x % 3 == 2) {
            if(ex.size() == 0) {
                i --; continue;
            }
            std::cout << "FIN " << ex[f % ex.size()] + 1 << "\n"; 
        } else {
            if(ex.size() == 0) {
                i --; continue;
            }
            std::cout << "DEL " << ex[f % ex.size()] + 1 << "\n";
        }
    }
    std::cout << "exit\n";
    return 0;
}
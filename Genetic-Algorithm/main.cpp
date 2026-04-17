#include <vector>
#include <chrono>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <array>
#include <tuple>
#include <unordered_set>

// 维度
const int N = 30;
// 值域
const int MAXN = 20;
// 20 二进制位数
const int binN = 5;
// f(x) 的最大值
const int MAXV = 12000;
// 种群容量
const int LIMIT = 100;
// 锦标赛一轮选择个数
const int ROUND_NUM = 10;
// 总共迭代的论述
const int GA_ROUND = 1000;
// 变异概率
const double P_MUTATE = 0.1;
// 每次杂交交换的染色体数量
const int SWAP_CNT = 10;

const int INF = 2147483647;
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
// 线性同余法生成随机数
inline int get_rand(int x = 2147483647) {
    seed = (int)(seed*48271ll%2147483647);
    return seed % x;
}
inline double get_real_rand() { return get_rand() * 1.0 / INF; }

// 5 位二进制 + 1 位符号
struct one {
    int value;
    bool symbol; // 符号 0:-, 1:+
    one(int v, bool b) : value(v), symbol(b) {}
    one() { value = get_rand(MAXN + 1), symbol = get_rand(2); }
};

struct Population_individual {
    std::array<one, N> v;
    // 缓存
    double fit; bool cala_ok;
    Population_individual() { cala_ok = false; }

    // 得到对应的染色体, return (序列, 二进制位, 是否是符号位)
    std::tuple<int, int, bool> get_idx_id_syb(int x) {
        int idx  = x / (binN + 1);
        int id   = x % (binN + 1);
        bool syb = false;
        if(id == binN) syb = true, id = -1;
        return {idx, id, syb};
    }
    // 突变
    void mutate() {
        int x = get_rand(N * (binN + 1));
        const auto &[idx, id, syb] = get_idx_id_syb(x);
        if(syb) {
            v[idx].symbol = !v[idx].symbol;
        } else {
            v[idx].value ^= (1 << id);
        }
    }
    // 单点交叉
    friend void hybrid(Population_individual &a, Population_individual &b, int x) {
        // 交叉的一条染色体
        const auto &[idx, id, syb] = a.get_idx_id_syb(x);
        if(syb) {
            std::swap(a.v[idx].symbol, b.v[idx].symbol);
        } else {
            int buf1 = (a.v[idx].value & (1 << id)), buf2 = (b.v[idx].value & (1 << id));
            // 保证在值域内
            if(a.v[idx].value + buf2 - buf1 <= MAXN && b.v[idx].value + buf1 - buf2 <= MAXN) {
                a.v[idx].value += buf2 - buf1, b.v[idx].value += buf1 - buf2;
            }
        }
    }
    // 多点交叉
    friend void hybrid(Population_individual &a, Population_individual &b, const auto &arr) {
        for(int x : arr) {
            assert(x < N * (binN + 1));
            hybrid(a, b, x);
        }
    }
    // 30 维求和
    int get_val() {
        int res = 0;
        for(const auto &[val, syb] : v) {
            int buf = val * (syb ? 1 : -1);
            res += buf * buf;
        }
        return res;
    }
    // 求适应度
    double get_fitness() {
        if(!cala_ok) {
            cala_ok = true;
            return fit = MAXV - get_val();
        }
        return fit;
    }
    bool operator<(Population_individual X) {
        return get_fitness() < X.get_fitness();
    }
    bool operator>(Population_individual X) {
        return get_fitness() > X.get_fitness();
    }
};

struct GA {
    int limit; // 种群容量
    double p;  // 突变概率
    std::vector<Population_individual> Populations;
    GA(int _limit, double _p) : limit(_limit), p(_p) {
        if(limit % 2) limit ++;
        while(Populations.size() < limit) {
            Populations.push_back(Population_individual());
        }
        sort(Populations.begin(), Populations.end());
    }
    // 迭代
    void iterate() {
        // 随机打乱
        std::random_shuffle(Populations.begin(), Populations.end());
        // 杂交
        for(int i = 0; i < limit; i += 2) {
            Populations.push_back(Populations[i]);
            Populations.back().cala_ok = false;
            Populations.push_back(Populations[i + 1]);
            Populations.back().cala_ok = false;
            int m = Populations.size() - 1;
            std::unordered_set<int> arrx;
            while(arrx.size() < SWAP_CNT) {
                arrx.insert(get_rand(N * (binN + 1)));
            }
            hybrid(Populations[m], Populations[m - 1], arrx);
            int p1 = get_real_rand();
            if(p1 < p) {
                Populations[m].mutate();
            }
            int p2 = get_real_rand();
            if(p2 < p) {
                Populations[m - 1].mutate();
            }
        }
        // 锦标赛筛选
        std::vector<Population_individual> Populations_next;
        while(Populations_next.size() < limit) {
            std::vector<Population_individual> joiners;
            for(int i = 0; i < ROUND_NUM; i ++) {
                joiners.push_back(Populations[get_rand(Populations.size())]);
            }
            int max_id = 0;
            for(int i = 1; i < ROUND_NUM; i ++) {
                if(joiners[max_id] < joiners[i]) {
                    max_id = i;
                }
            }
            Populations_next.push_back(joiners[max_id]);
        }
        std::swap(Populations_next, Populations);
        sort(Populations.begin(), Populations.end());
    }
    // 中位数
    int get_mid() {
        return Populations[limit / 2].get_val();
    }
    // 平均数
    double get_avg() {
        double all = 0;
        for(auto i : Populations) {
            all += i.get_val();
        }
        return all / limit;
    }
};

int main () {
    std::cout << std::fixed << std::setprecision(6);
    freopen("res", "w", stdout);
    GA ga(LIMIT, P_MUTATE);
    for(int _ = 0; _ <= GA_ROUND; _++) {
        std::cout <<  _ << " " << ga.get_avg() << "\n"; 
        // std::cout << "第" << _ << "代:\n中位数: " << ga.get_mid() << "   平均数：" << ga.get_avg() << "\n种群中的个体值: "; 
        // for(auto x : ga.Populations) {
        //     std::cout << x.get_val() << " ";
        // }
        // std::cout << "\n\n";
        ga.iterate();
    }
}
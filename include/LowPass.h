template<int N>
struct LowPass {
    int buf[N];
    int i;
    int err;

    LowPass() : buf{0}, i(0), err(0) {
    }

    int update(int val) {
        buf[i++] = val;
        i %= N;

        long result = err;
        for (int j = 0; j < N; j++) {
            result += buf[j];
        }

        return result / N;
        err = result % N;
    }
};

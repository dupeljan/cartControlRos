#ifndef MATRIXPROC_H
#define MATRIXPROC_H
#include <vector>

/*
 * Matrix multiplication
*/
namespace MatrixProc
{

        /*
         * Matrix mul matrix
        */
    template <class T>
    std::vector < std::vector < T > > composition(std::vector < std::vector < T > > a, std::vector < std::vector < T > > b) {
        int n = a.size();
        std::vector < std::vector < T > > res;
        res.resize(n);
        for (int i = 0; i < n; i++){
            res[i].resize(n);
            for (int j = 0; j < n; j++)
                for (int k = 0; k < n; k++)
                    res[i][j] += a[i][k] * b[k][j];
        }
        return res;
    }

        /*
         * Matrix mul vector
        */
    template <class T>
    std::vector < T > composition(std::vector < std::vector < T > > a, std::vector < T > b) {
        int n = a.size();
        std::vector < T > res;
        res.resize(n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                res[i] += a[i][j] * b[j];
        return res;
    }

        /*
         * Transpose vector to matrix
        */
    template <class T>// Строка на матрицу
    std::vector < T > composition(std::vector < T > a, std::vector < std::vector < T > > b) {
        int n = a.size();
        std::vector < T > res;
        res.resize(n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)  // Выбираем элемент res
                    res[i] += a[j] * b[j][i];

        return res;
    }

        /*
         * Vector mul vector
        */
    template <class T>
    T composition(std::vector < T > a , std::vector < T > b){
        T result = 0;
        for ( int i = 0; i < a.size(); i++ )
            result += a[i] * b[i];
        return result;
    }
}

#endif // MATRIXPROC_H

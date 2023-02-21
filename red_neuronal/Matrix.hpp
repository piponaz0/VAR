#include <memory>
#include <algorithm>
#include <initializer_list>
#include <cassert>
#include <random>
#include <iostream>
#include <vector>

using namespace std;

template<typename T>
struct Matrix {

    struct random_tag{};
    struct zero_tag{};

    inline static random_tag random{};
    inline static zero_tag zero{};

    using value_type = T;
    using storage_type = std::unique_ptr<value_type[]>; 
    using initlist_type = std::initializer_list<value_type>;
    using initmatrix_type = std::initializer_list<initlist_type>;

    inline static value_type getRandom(value_type const min, value_type const max){
        static std::random_device device{};
        static std::mt19937 rng{device()};
        return std::uniform_real_distribution{min,max}(rng);
    }

    Matrix() 
        :   Matrix(1,1)
    {
    }

    Matrix(std::size_t filas, std::size_t columnas,random_tag)
        :   filas_{filas}, columnas_{columnas}
    {
        for(std::size_t i{};i<tam_;i++){
            data_[i] = getRandom(-5.0,5.0);
        }
    }

    Matrix(std::size_t filas, std::size_t columnas,zero_tag)
        :   filas_{filas}, columnas_{columnas}
    {}

    Matrix(std::size_t filas, std::size_t columnas) : Matrix(filas,columnas,zero_tag{})
    {}

    Matrix(initmatrix_type R)
        :   filas_{R.size()}, columnas_{ (filas_>0) ? R.begin()->size() : 0} 
    {
        std::size_t n{};
        for(auto& row : R) {
            assert(row.size() == columnas_);
            for(auto item : row) {
                data_[n] = item;
                ++n;
            }
        }
    }

    Matrix(Matrix<value_type> const& R) : 
       filas_{R.filas_}, columnas_{R.columnas_}
    {
        //Para copiar el contenido de la matriz. Tiene 3 parametros: donde empiezo a leer, donde termino de leer, y donde voy a copiar
        //el .get() es para obtener el puntero de la clase unique_ptr y que pueda compilarS
        std::copy(R.data_.get() + 0, R.data_.get() + tam_, data_.get() + 0);
    }

    Matrix(vector<value_type> inputs){
        int columnas = inputs.size();
        Matrix m(1, columnas);
        
        for (int i = 0; i < columnas; i++){
            m.data_[i] = inputs[i];
        }
        *this = m;
    }

    value_type& operator()(std::size_t const fila, std::size_t const columna) noexcept {
        return data_[fila * columnas_ + columna];
    }

    value_type const& operator()(std::size_t const fila, std::size_t const columna) const noexcept {
        return data_[fila * columnas_ + columna];
    }

    value_type& at(std::size_t const fila, std::size_t const columna) {
        auto value = fila * columnas_ + columna;
        if(value >= tam_) {
            throw  std::__throw_out_of_range("Matrix::at : Invalid reference");
        }
        return data_[value];
    }

    void insertColumnleft(value_type const initval){
        auto const newSize = (columnas_+1)*filas_;
        storage_type newData { std::make_unique<value_type[]>(newSize)};
        /*PRUEBA EN SENCILLO
        newData[0] = initval;
        std::copy(&data_[0],&data_[0+columnas_],&newData[1])
        newData[1+columnas_] = initval;
        std::copy(&data_[columnas_],&data_[columnas_+columnas_-1],&newData[2+columnas_])*/
        //
        auto data_ptr = data_.get();
        auto newdataptr = newData.get(); //puntero para luego copiar
        for(std::size_t i{};i<filas_;i++){
            *newdataptr = initval;
            ++newdataptr;
            std::copy(data_ptr,data_ptr+columnas_,newdataptr);
            data_ptr += columnas_;
            newdataptr += columnas_;
        }


        
        data_ = std::move(newData);
        tam_ = newSize;
        ++columnas_;

    }

    //tipo funcion que hace una funcion que devuelve el velue type
    using functionPtr_type = value_type (*)(value_type);

    void applyFunction(functionPtr_type function){
        for(std::size_t i{};i<tam_;i++){
            data_[i] = function(data_[i]);
        }
    }

    void applyDerivadaTanh(){
        for(std::size_t i{};i<tam_;i++){
            data_[i] = 1 - std::pow(std::tanh(data_[i]),2);
        }
    }

    friend Matrix operator*(Matrix<value_type> const& L, Matrix<value_type> const& R) {
        if(L.columnas() != R.filas()) throw  std::out_of_range("Matrix::multiply : Invalid size");

        Matrix<value_type> res(L.filas(),R.columnas());

        for(std::size_t i{}; i < res.filas(); ++i) {
            for(std::size_t j{}; j < res.columnas(); ++j) {
                res(i,j) = 0;
                
                for(std::size_t k{}; k < L.columnas(); ++k) {
                    res(i,j) += L(i,k) * R(k,j);
                }
            }
        }
        return res;
    }

    friend std::ostream& operator<<(std::ostream& out, Matrix<value_type>const& M){
        for(std::size_t r{};r<M.filas();r++){
            for(std::size_t c{};c<M.columnas();c++){
                out << M(r,c) << "\t";
            }
            out << "\n";
        }
        return out;
    }
    // El doble & es un puntero doble: una referencia a un valor temporal
    Matrix& operator=(Matrix<value_type>&& R) {
        filas_ = R.filas_;
        columnas_ = R.columnas_;
        tam_ = R.tam_;
        data_ = std::move(R.data_);
        R.filas_ = R.columnas_ = R.tam_ = 1;
        R.data_ = std::make_unique<value_type[]>(1);

        return *this;
    }

    //Aqui se reutiliza el operador= temporal que está definido justo arriba
    Matrix& operator=(Matrix<value_type> const& R) {
        *this = Matrix<value_type> {R};
        return *this;
    }
    /*
    void print() {
        for (int i = 0; i < tam_; i++){
            cout << data_[i] << " ";
            if((i + 1) == columnas_){
                cout << "\n";
            }
        }
        cout << "\n";
    }*/

    void insertFirstColumnOne() {
        storage_type dataAux_ {std::make_unique<value_type[]>(tam_ + filas_)};
        dataAux_[0] = 1;
        std::size_t i{}, j{};
        ++i;

        while (j < tam_) {
            if(j%columnas_ == 0){
                dataAux_[i] = 1;
                ++i;
            }
            dataAux_[i] = data_[j];
            ++j;
            ++i;
        }
        data_ = dataAux_;
        tam_ = tam_ + filas_;
        columnas_ += 1;
   
 }

    std::size_t filas() const noexcept {return filas_;}
    std::size_t columnas() const noexcept {return columnas_;}
    std::size_t tam() const noexcept {return tam_;}

    public:
    std::size_t filas_{}, columnas_{}, tam_{filas_*columnas_};
    storage_type data_ { std::make_unique<value_type[]>(tam_)};

};



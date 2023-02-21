#include "Matrix.hpp"
#include <cmath>
#include <vector>
#include <sstream>
#include <fstream>
#include <ctime> 
constexpr int n_game_status = 38;
constexpr int sampleSize = 7004;

template <typename DataType>
struct Net {
    using value_type = Matrix<DataType>;
    using storage_type = std::unique_ptr<value_type[]>; 
    using initlist_type = std::initializer_list<DataType>;

    value_type getInputData(const string& str){
        vector <DataType> inputs;
        
        std::istringstream ss(str);
        std::string token;
        int matPos = 1;
        double num;
        

        while(std::getline(ss,token,';')){
            if(matPos <= n_game_status - 1) {
                num = stod(token);
                inputs.push_back(num);
            }
            matPos++;
        }
        value_type inputGame{inputs};
        return inputGame;
    }

    value_type getOutputData(const string& str){
        vector <DataType> outputs;
        
        std::istringstream ss(str);
        std::string token;
        int matPos = 1;
        double num;
        

        while(std::getline(ss,token,';')){
            if(matPos > n_game_status-1){
                num = stod(token);
                outputs.push_back(num);
            }
            matPos++;
        }
        value_type outputGame{outputs};
        return outputGame;
    }

    void readGameFile(string filename,vector<value_type> &Inputs, vector<value_type> &Outputs){
        int dataAnalyzed = 0;
        string line="";
        fstream fileData(filename.c_str());
        value_type gameInput, gameOutput;

        while(dataAnalyzed < sampleSize){
            getline(fileData,line);
            gameInput = getInputData(line);
            gameOutput = getOutputData(line);
            //std::cout << "GAMEINPUT: " << gameInput << "\n";
            //std::cout << "GAMEOUTPUT: "  << gameOutput << "\n";

            Inputs.push_back(gameInput);
            Outputs.push_back(gameOutput);

            dataAnalyzed++;
            line = "";
        }
    }

    Net(std::size_t const inputs, initlist_type arch, std::size_t const outputs) 
        : layers_{arch.size() + 1}
    {
        std::size_t i{};
        auto rows = inputs;
        for(auto& cols : arch) {
            //inputs = rows +1 añadimos el peso para el bias
            data_[i] = value_type(rows+1, cols, value_type::random);
            i++;
            rows = cols;
        }
        data_[i] = value_type(rows+1, outputs, value_type::random);
    }

    Net(const Net<DataType>&R) 
        : layers_{R.layers_}
    {
        std::copy(R.data_.get() + 0, R.data_.get() + layers_,data_.get()+0);
    }

    //no discard salta un warnign de no uso
    [[nodiscard]] value_type feedForward(value_type const& inputs) const
    {
        //TODO Coge el input y multiplica por la primera capa. Luego la salida la multiplica por la segunda capa, etc.
        //Acuerdate de poner el x0 inicial para mover la recta y que no pase siempre por el origen
        value_type outputs{inputs};
        //cout << outputs << "\n";
        for(std::size_t l{}; l < layers_; l++){
            outputs.insertColumnleft(1.0);
            outputs = outputs * data_[l];  //Multiplicamos por cada capa las entradas
            signal_[l] = outputs;
            outputs.applyFunction(std::tanh); //Aplicamos la tangente hiperbólica a nuestra señal (Función FI)
        }
        return outputs;
    }

    DataType evaluateNet(value_type forwardOuts, value_type Outputs){
        return this->getError(forwardOuts,Outputs);
    }

    Net<DataType> trainNet(vector<Matrix<double>> const& Inputs, vector<Matrix<double>> const& Outputs){
        constexpr size_t MAX_NET = 20;
        Net<DataType> net = *this;
        value_type forwardOuts;
        DataType bestError;
        DataType errorMedio;
        DataType auxError;
        value_type outs;
        
        for(size_t i{}; i < MAX_NET; i++){
            Net<DataType> newNet{37 ,{37, 37}, 1};

            for (uint j = 0; j < Inputs.size(); j++){
                outs = newNet.feedForward(Inputs[j]);
                auxError += newNet.evaluateNet(outs,Outputs[j]);
            }
            errorMedio = auxError/Inputs.size();
            if (i == 0){
                bestError = errorMedio;
            }
            if(errorMedio < bestError){
                bestError = errorMedio;
                net = newNet;
            }
        }

        DataType error;
 
        unsigned t0, t1;

        t0=clock();

        for (uint j = 0; j < Inputs.size(); j++){
            for (uint l = 0; l < 8; l++){
                outs = net.feedForward(Inputs[j]);
                net.actualizarPesos(net.deltaSalida(net.getErrorDeriv(outs,Outputs[j])),Inputs[j]);
            }
            error += net.evaluateNet(outs,Outputs[j]);
        }

        t1 = clock();
        
        std::cout << "\nENTRENAMIENTO TERMINADO "<< endl;
        std::cout << "El error anterior: " << bestError << "\n";
        std::cout << "El error después de entrenar: " << error/Inputs.size() << "\n";
        double time = (double(t1-t0)/CLOCKS_PER_SEC);
        cout << "Execution Time: " << time << endl;

        return net;
    }

    Net& operator=(Net<DataType>&& R){
        this->data_ = std::move(R.data_);
        this->layers_ = R.layers_;
        return *this;
    }
    
    Net& operator=(Net<DataType> const& R){
        *this = Net<DataType>{R};
        return *this;
    }

    double getError(value_type const& obtenido, value_type const& esperado) const{ //Este calculo es solo para comprobar
        double error{};

        for(std::size_t i{};i < obtenido.tam();i++){
            error += pow(obtenido.data_[i] - esperado.data_[i],2);
        }
        error = error/obtenido.tam();

        return error;
    }

    double getErrorDeriv(value_type const& obtenido, value_type const& esperado) const{ //Calculo del error medio
        double error{};

        for(std::size_t i{};i < obtenido.tam();i++){
            error += 2 * (obtenido.data_[i] - esperado.data_[i]); //sumatorio de los errores
        }
        error = error/obtenido.tam();
        
        return error;
    }

    [[nodiscard]]value_type deltaSalida(float const& error) const{
        value_type outs{signal_[layers_ -2]};

        for(std::size_t i{}; i < outs.tam(); i++){
            outs.data_[i] = error * (1 - std::pow(std::tanh(outs.data_[i]),2)); //Derivada de la tangente
        }
        delta_[layers_ -1] = outs;
        return outs;
    }

    void deltaOcultas(value_type const& inputs)const{
        
        value_type outs{inputs};
        float s = 0;

        for(int l = layers_ - 2; l > - 1; l--){ //Vamos hacia la primera capa
            if(l != 0){
                value_type outs{signal_[l - 1]}; //Obtenemos x * w de la capa anterior
            }
            std::size_t j{};
            s = 0;
            for(std::size_t i{data_[l + 1].columnas()}; i < data_[l + 1].tam(); i++){
                auto wij = data_[l + 1].data_[i];
                auto delta = delta_[l + 1].data_[j];
                s +=  wij * delta;  //Sumatorio pesos + deltas
                j++;
                if(j == delta_[l + 1].tam()){
                    j = 0;
                }
            }
            for(std::size_t i{}; i < outs.tam(); i++){
                outs.data_[i] = s * (1 + std::pow(std::tanh(outs.data_[i]), 2)); //Lo multiplicamos por la derivada de la función no lineal
            }
            delta_[l] = outs;
        }
    }
    void actualizarOcultas(){
        for(int l = layers_ - 2; l >- 1; l--){
            value_type s{signal_[l + 1]}; //Obtengo las señales de entrada
            s.applyFunction(std::tanh); //Obtengo s
            value_type outs(s.columnas_, s.filas_);

            for(std::size_t i{}; i < s.tam_; i++){
                outs.data_[i] = s.data_[i];
            }
            value_type calculoSD =  outs * delta_[l + 1]; //s * delta
            std::size_t j{};

            for(std::size_t i{}; i < data_[l].tam_; i++){
                data_[l].data_[i] = data_[l].data_[i] - (0.01 * calculoSD.data_[j]);  //w = w - factor * delta * s
                j++;
                if(j == calculoSD.tam_){
                    j = 0;
                }
            }
        }
    }
    

    //Calculamos la actualización del peso de la última capa.
    //Después calculamos la delta de las ocultas y actualizamos pesos de las ocultas.
    void actualizarPesos(value_type const& deltaOut, value_type const& inputs){
        value_type s{signal_[layers_ - 2]}; //Obtengo la entrada de la última capa
        s.applyFunction(std::tanh); //Obtengo s
        value_type outs(s.columnas(), s.filas());

        for(std::size_t i{}; i < s.tam(); i++){
            outs.data_[i] = s.data_[i];
        }
        value_type calculoSD =  outs * deltaOut; //s * delta
        for(std::size_t i{}; i < data_[layers_- 1].tam(); i++){
            data_[layers_ - 1].data_[i] = data_[layers_ - 1].data_[i] - (0.01 * calculoSD.data_[i]);  //w = w - factor * delta * s
        }
        deltaOcultas(inputs);
        actualizarOcultas();
    }

    friend std::ostream& operator<<(std::ostream& out, Net<DataType> const& Net){
        ofstream file("netWeights.txt");
        out<<"(NET LAYERS: " << Net.layers_ << ")\n";
        for(std::size_t l{}; l < Net.layers_; l++){
            out << "Layer" << 1 + l <<" (In: ";
            out << Net.data_[l].filas() << " Out: "<< Net.data_[l].columnas() <<")\n";
            
            for(std::size_t i{}; i < Net.data_[l].tam(); i++){
                out<< Net.data_[l].data_[i] << ", "; 
            }
            out<<"\n\n";

            if(l+1 == Net.layers_){
                file << "Layer" << 1 + l << "\n";
                for(std::size_t i{1}; i <= Net.data_[l].tam(); i++){
                    if(i%16!=0){
                        file << Net.data_[l].data_[i-1] << ", ";
                    }else if(i!=0){
                        file <<Net.data_[l].data_[i-1]<< "\n";
                    }else if(i==0){
                        file << Net.data_[l].data_[i-1] << ", ";
                    }
                    
                    
                }
            }
        }
        return out;
    }

public:
    std::size_t layers_{1};
    storage_type data_ { std::make_unique<value_type[]>(layers_)};
    storage_type signal_{std::make_unique<value_type[]>(layers_)};
    storage_type delta_{std::make_unique<value_type[]>(layers_)};

};

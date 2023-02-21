#include "Net.hpp"
#include <iostream>
#include <stdexcept>
#include <cstdlib> 
#include <ctime> 
#include <memory>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

void run(){
   Matrix<double>input0{
      { 0,0 }
   };
   Matrix<double>input1{
      { 0,1 }
   };
   Matrix<double>input2{
      { 1,0 }
   };
   Matrix<double>input3{
      { 1,1 }
   };
   Matrix<double>output0{
      {0},
   };
   Matrix<double>output1{
      {1},
   };
   Matrix<double>output2{
      {1},
   };
   Matrix<double>output3{
      {0},
   };

   //vector<Matrix<double>> inputs ={ input0,input1,input2,input3 };
   //vector<Matrix<double>> outputs = { output0, output1, output2, output3};

   vector<Matrix<double>> inputs;
   vector<Matrix<double>> outputs;

   Net<double> neuralNet{37 ,{37,37}, 1};
   //std::cout << neuralNet << "\n";

   //neuralNet.getGameData(str);

   neuralNet.readGameFile("turtlebot3_datos.csv",inputs,outputs);
   neuralNet.trainNet(inputs,outputs);
   std::cout << neuralNet;
   
   
   /*for (uint i = 0; i < inputs.size(); i++){
      
      
      cout << "Input "<< i <<endl;
      cout << inputs[i] << "\n";
      [[maybe_unused]]auto outs = neuralNet.feedForward(inputs[i]);
      std::cout << "Salida obtenida: " << outs;
      std::cout << "Salida esperada: " << outputs[i];
      std::cout << "El error: " << neuralNet.getError(outs,outputs[i]) << "\n";
      neuralNet.actualizarPesos(neuralNet.deltaSalida(neuralNet.getErrorDeriv(outs,outputs[i])),inputs[i]);
      cout << "\n";
   }*/
    
}
int main(){
   try{
      run();
   }
   catch(std::exception const& e){
      std::cout <<"[[EXCEPTION]]:\n";
      std::cout <<"   "<<e.what()<<"\n\n";
      return 1;
   }
   return 0;
}

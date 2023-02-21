#include <fstream>
#include <iostream>
#include <vector>
#include <string>

using namespace std;

void split(std::string str, std::string splitBy, std::vector<std::string>& tokens)
{
    /* Store the original string in the array, so we can loop the rest
     * of the algorithm. */
    tokens.push_back(str);

    // Store the split index in a 'size_t' (unsigned integer) type.
    size_t splitAt;
    // Store the size of what we're splicing out.
    size_t splitLen = splitBy.size();
    // Create a string for temporarily storing the fragment we're processing.
    std::string frag;
    // Loop infinitely - break is internal.
    while(true)
    {
        /* Store the last string in the vector, which is the only logical
         * candidate for processing. */
        frag = tokens.back();
        /* The index where the split is. */
        splitAt = frag.find(splitBy);
        // If we didn't find a new split point...
        if(splitAt == std::string::npos)
        {
            // Break the loop and (implicitly) return.
            break;
        }
        /* Put everything from the left side of the split where the string
         * being processed used to be. */
        tokens.back() = frag.substr(0, splitAt);
        /* Push everything from the right side of the split to the next empty
         * index in the vector. */
        tokens.push_back(frag.substr(splitAt+splitLen, frag.size()-(splitAt+splitLen)));
    }
}

int main () {
    string entrada = "/home/pedro/universidad/var/practicas/turtlebot3_original.txt";
    string salida = "/home/pedro/universidad/var/practicas/turtlebot3_limpio.txt";
    fstream file_out;
    fstream file_in;
    string linea;
    vector<string> listaNumeros;

    file_out.open(salida, std::ios_base::out);
    file_in.open(entrada, std::ios_base::in);

    if(file_in.is_open()) {
        while(getline(file_in,linea)){
            split(linea, ";", listaNumeros);
            if(file_out.is_open()) {
                for(std::string s : listaNumeros) {
                    if (s != "inf") {
                        string::size_type sz;
                        float numero = stof(s,&sz);
                        if(numero > 0 && numero < 0.15) {
                            s = "inf";
                        }
                    }
                    file_out << s << ';'; 
                }
                listaNumeros.clear();
            file_out << endl;
            }
        }
    } else {
        cout << "Error en fichero de entrada" << endl;
    }
    file_in.close();
    file_out.close();
    
}
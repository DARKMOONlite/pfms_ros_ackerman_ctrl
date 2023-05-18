#include "analysis.h"
#include <vector>
#include <iostream> // Only here for showing the code is working
#include <cctype>

namespace analysis {

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
unsigned int countCharacters(std::string sentence){
    unsigned int count = 0;
    for(unsigned int i = 0; i < sentence.length(); i++){
        //if(sentence[i] != ' '){ // if it not a space //! nvm they want spaces to be included
            count++;
        //if(count)}


    }

    return count;
}

//! @todo
//! TASK 5 - Refer to README.md and the Header file for full description
int getNumber(std::string sentence){
    unsigned int num = 0;
    std::vector<char> number;
    
    
    
    for(unsigned int i = 0; i < sentence.length(); i++){
        if(isdigit(sentence[i])){
            number.push_back(sentence[i]);
        }
        if(isalpha(sentence[i]) && number.size()>0){

            break;
        }

    }
    std::string buffer_string(number.begin(), number.end());
    num = std::stoi(buffer_string);
    
    return num;

}

}

#include "MongoDBClient.h"
#include <iostream>
#include <string>

int main() {
    MongoDBClient dbManager;  
    
    dbManager.displayDocuments();

    std::cout << "\nEnter an ID or Timestamp to download the corresponding mesh data:" << std::endl;
    std::string input;
    std::getline(std::cin, input);  

    dbManager.downloadMeshData(input);

    return 0;
}

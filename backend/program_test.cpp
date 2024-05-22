#include <iostream>
#include "MongoDBClient.h"

void testConnection() {
    try {
        MongoDBClient dbManager;
        dbManager.displayDocuments();
        std::cout << "Connection Test Passed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Connection Test Failed: " << e.what() << std::endl;
    }
}

void testInvalidInput() {
    try {
        MongoDBClient dbManager;
        std::string invalidInput = "valid_id"; // Presumably, this should be a test input that is known to be invalid
        dbManager.downloadMeshData(invalidInput);
        std::cout << "Invalid Input Test Passed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Invalid Input Test Failed: " << e.what() << std::endl;
    }
}

void testValidInput() {
    try {
        MongoDBClient dbManager;
        std::string validInput = "6613f3a8b606c54329950922"; // Ensure this is actually a valid ID in your database
        dbManager.downloadMeshData(validInput);
        std::cout << "Valid Input Test Passed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Valid Input Test Failed: " << e.what() << std::endl;
    }
}

int main() {
    testConnection();
    testInvalidInput();
    testValidInput();
    return 0;
}
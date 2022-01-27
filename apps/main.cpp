#include <iostream>

/*
 * Program begins and ends execution with a call to main.
 */
int main(int argv, char* argc[]) 
{
    // Unhandled exceptions are caught by the try-catch clause, to ensure the program
    // exits cleanly.
    try
    {
        std::cout << "Hello world!" << std::endl;

        return EXIT_SUCCESS;
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled exception caught: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unhandled exception caught." << std::endl;
    }

    return EXIT_FAILURE;
}

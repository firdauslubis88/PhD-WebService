#include <iostream>

int main(int argc, char** argv)
{
	int input = 0;
	if (argc > 1)
	{
		sscanf(argv[1], "%d", &input);
		std::cout << input << std::endl;
	}

}
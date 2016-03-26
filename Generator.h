#ifndef GENERATOR_H
#define GENERATOR_H
#include "Includes.h"
/*
class Generator
{
	unique_ptr<std::default_random_engine> pGenerator;
	unique_ptr<std::uniform_int_distribution<int>> pDistribution;

public:
	Generator(int a = 0,int b = 255)
	{
		pGenerator.reset(new  std::default_random_engine());
		pDistribution.reset(new std::uniform_int_distribution<int>(a,b));
	}
	int generate()
	{
		return (*pDistribution)(*pGenerator);
	}
	template<typename T>
	vector<T> generateVector(size_t length)
	{//генерация мусора для передачи по каналу
		pGenerator->seed(time(NULL));

		vector<T> rubish(length);
		for (T& el : rubish)
			el = (T)generate();
		
		return rubish;
	}

};
*/
class Generator
{
public:
	template<typename T>
	vector<T> generateVector(size_t length)
	{//генерация мусора для передачи по каналу
		srand(time(NULL));
		vector<T> rubish(length);
		for (T& el : rubish)
			el = rand();

		return rubish;
	}
};
#endif //GENERATOR_H


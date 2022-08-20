/*
Parallel bitonic sorting algorithm.
Works with any 2^n vector<int>.
Implements all the additional features of the task.

Leevi Hokkanen, 050253975
Noora Sassali, H299753
Olivia Saukonoja, K438991*/


#include <iostream>
#include <vector>
#include <cstdint>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <math.h>
#include <chrono>
#include <algorithm>
#include <random>

// Blocks execution, if no threads are available.
std::condition_variable cv;
// Used for blocking thread task assignment with cv.
std::mutex iteration_mutex;
// Used to index and check the amount of threads available.
// If the value is 0, then none are available.
unsigned int semaphore = 0;

/**
 * @brief updateSemaphore
 * Removes one available thread.
 */
void semaphoreDown()
{
    semaphore -= 1;
}

/**
 * @brief semaphoreUp
 * Adds one available thread.
 */
void semaphoreUp()
{
    semaphore += 1;
}

/**
 * @brief vectorToStr
 * Turns the content of the vector into a printable string.
 * @param vec Vector to be stringified.
 * @return str Stringified vector.
 */
std::string vectorToStr(std::vector<int> vec)
{
    std::string str = "{";

    for(unsigned int i=0; i<vec.size(); i++)
    {
        str += std::to_string(vec.at(i));

        if(i<vec.size()-1)
        {
            str += ",";
        }
    }
    str += "}";

    return str;
}

/**
 * @brief isBiggerThan
 * Checks, whether num at index i is bigger than num at index j.
 * If yes, the position of these nums will be swapped.
 * @param a: An integer
 * @param b: An integer
 * @param vec: A vector containing integers.
 */
void isBiggerThan(unsigned int i, unsigned int j, std::vector<int>& vec)
{
    int a = vec.at(i);
    int b = vec.at(j);

    if(a>b)
    {
        vec.at(i) = b;
        vec.at(j) = a;
    }
}

/**
 * @brief isSmallerThan
 * Checks, whether num at index i is smaller than num at index j.
 * If yes, the position of these nums will be swapped.
 * @param a: An integer
 * @param b: An integer
 * @param vec: A vector containing integers.
 */
void isSmallerThan(unsigned int i, unsigned int j, std::vector<int>& vec)
{
    int a = vec.at(i);
    int b = vec.at(j);

    if(a<b)
    {
        vec.at(i) = b;
        vec.at(j) = a;
    }
}

/**
 * @brief threadsAvailable
 * Checks whether there are any threads available.
 * @return true, if yes. false, if no.
 */
bool threadsAvailable()
{
    return semaphore != 0;
}

/**
 * @brief joinThreads
 * Joins the threads.
 * @param threads: A vector of threads.
 */
void joinThreads(std::vector<std::thread>& threads)
{
    for (auto& th : threads)
    {
        if(th.joinable())
        {
            th.join();
            semaphoreUp();
        }
    }
    iteration_mutex.unlock();
    cv.notify_one();
}

/**
 * @brief bitonicSort
 * Sorts the given vector with bitonic sort algorithm.
 * Uses semi-bitonic if the vector has odd number of elements. (Last one will be sorted separately)
 * @param vec: A vector that needs to be sorted.
 * @param threads: Threads that are used for sorting the vector.
 * @return vec: A sorted vector.
 */
std::vector<int> bitonicSort(std::vector<int> vec, std::vector<std::thread> threads)
{
    // I profoundly apologise for the awful amount of comments and variables
    // that make this function horrible to read.


    // The amount of comparator direction changes in a stage.
    unsigned int compsForStage = vec.size()/2;

    // Bitonic steps needed for sorting.
    unsigned int stepsNeeded = log2(vec.size());

    // Tells the max index difference between compared num1 and num2.
    unsigned int index_diff_Max = 1;

    // Max amount of num1s without indexLeap (only singular steps).
    unsigned int noLeaps_Max = 1;
    unsigned int noLeaps = 1;

    // Tells the index difference between each num1 after singular leaps.
    unsigned int indexLeap_Max = 1;


    // Iterate steps.
    for(unsigned int step = 1; step<=stepsNeeded; step++)
    {
        // Bitonic stages needed.
        unsigned int stagesNeeded = step;

        // Iterate stages.
        for (unsigned int stage = 1; stage<=stagesNeeded; stage++)
        {
            // Used for calculating how many steps, or how long of an
            // index leap there is between numbers on different stages.
            unsigned int stageDiff = pow(2,stage-1);

            // Same thing, but only for the first handled stage of step.
            unsigned int stageDiff_Max = pow(2,stagesNeeded-1);

            noLeaps = noLeaps_Max/stageDiff;

            // Tells when comparator swaps comparison direction.
            bool swap_flag = true;

            unsigned int index1 = 0;
            unsigned int index2 = 0;

            // Iterate comparisons for a stage.
            for(unsigned int i=0; i<compsForStage; i++)
            {

                index2 = index1 + index_diff_Max/stageDiff;

                // Locks the loop, if not threads are available.
                {
                    std::unique_lock<std::mutex> lk(iteration_mutex);
                    cv.wait(lk, [&threads]
                    {
                        bool available = threadsAvailable();

                        // Joins all the threads, if none are available.
                        if(available == false)
                        {
                            std::thread tr(joinThreads,std::ref(threads));
                            tr.detach();
                        }

                        return available;
                    });
                }

                unsigned int threadIndex = semaphore - 1;
                semaphoreDown();

                // Checks comparison direction and assigns comparison for a thread.
                if(swap_flag == true)
                {
                    threads.at(threadIndex) = std::thread(isBiggerThan,index1,index2,std::ref(vec));
                }
                else
                {
                    threads.at(threadIndex) = std::thread(isSmallerThan,index1,index2,std::ref(vec));
                }


                index1 += 1;

                // Checks whether more singular steps will be made
                // with index1.
                if(noLeaps == 1 || stagesNeeded == stage)
                {
                    noLeaps = noLeaps_Max / stageDiff;
                    index1 += indexLeap_Max / stageDiff;

                    // Every block splitting index is divisible
                    // with first stageDiff of step.
                    if((i+1) % stageDiff_Max == 0)
                    {
                        if(swap_flag == true)
                        {
                            swap_flag = false;
                        }
                        else
                        {
                            swap_flag = true;
                        }
                    }
                }
                else
                {
                    noLeaps -= 1;
                }

            }

            // Joins all threads before next step.
            joinThreads(std::ref(threads));
            semaphore = threads.size();
        }

        // After every stage the value of index difference between compared
        // numbers, singular steps, and index leaps will be halved after each
        // done stage of step.
        // First stage will always have those same values double the value of next.
        noLeaps_Max *= 2;
        index_diff_Max *= 2;
        indexLeap_Max *= 2;
    }

    return vec;
}


int main()
{

    // A lot of the terminology here is based of this article:
    // https://www.geeksforgeeks.org/bitonic-sort/

    // A definitions, for clarity.

    // Stage = A single round comparisons between vector elements.
    // Amount doubled after each step.

    // Block = A chunk of elements in a vector, that have a comparator
    // pointing to same direction in a stage. Amount halved after each step.

    // Step = A series of stages.

    // Singlar Steps = Amount of comparison points with index difference of 1.



    // Feel free to modify the following vector to any other 2^n vector. (n>0)
    std::vector<int> sortMe;

    // Define vector numbers. Avoid making the vector too large.
    // It crashes the program.
    for(int i=1; i<=pow(2,4); i++)
    {
        sortMe.push_back(i);
    }

    // Shuffling of the numbers.
    auto rng = std::default_random_engine {};
    std::shuffle(std::begin(sortMe), std::end(sortMe), rng);

    // Makes the vector of threads.
    unsigned int thread_count = 8;
    std::vector<std::thread> threads;
    threads.reserve(thread_count);

    semaphore = thread_count;

    for(unsigned int i=0; i<thread_count; i++)
    {
        std::thread thread;
        threads.push_back(std::move(thread));
    }


    // Sorting with Bitonic:
    auto startBitonic = std::chrono::steady_clock::now();
    std::vector<int> sortedBitonic = bitonicSort(sortMe, std::move(threads));
    auto endBitonic = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_Bitonic = endBitonic-startBitonic;


    // Sorting with STL-sort (Either quick-sort or insertion-sort)
    std::vector<int> sortedStl = sortMe;
    auto startStl = std::chrono::steady_clock::now();
    std::sort(sortedStl.begin(),sortedStl.end());
    auto endStl = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_Stl = endStl-startStl;



    // Printing of the vectors:

    std::cout << "Unsorted vector:" << std::endl;
    std::cout << vectorToStr(sortMe) << std::endl;

    std::cout << std::endl;

    std::cout << "Bitonic sorted vector:" << std::endl;
    std::cout << vectorToStr(sortedBitonic) << std::endl;
    std::cout << "Sorting time: " << elapsed_Bitonic.count() << std::endl;

    std::cout << std::endl;

    std::cout << "STL sorted vector:" << std::endl;
    std::cout << vectorToStr(sortedStl) << std::endl;
    std::cout << "Sorting time: " << elapsed_Stl.count() << std::endl;

    std::cout << std::endl;

    std::cout << "This implementation of Bitonic seems to be "
                 "significantly slower than single threaded stl-sort." << std::endl;

    std::cout << "Parallel implementation seems to receive its full effect "
                 "with larger vectors." << std::endl;

    return 0;
}

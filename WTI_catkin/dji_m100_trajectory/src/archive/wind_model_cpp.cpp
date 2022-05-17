#define PY_SSIZE_T_CLEAN
#include <Python.h>
float windspeed;
int
main(int argc, const char *argv[])
{   const char* n_argv[] = { "/wind_generation.exe", "windy_model", "wind_velocity"};
    argv = n_argv;
    argc=3;
    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;
    int i;
    Py_Initialize();
    pName = PyUnicode_DecodeFSDefault(argv[1]);
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);
        pFunc = PyObject_GetAttrString(pModule, argv[2]);
        /* pFunc is a new reference */
        if (pFunc && PyCallable_Check(pFunc)) {
            pArgs = PyTuple_New(argc - 3);
            for (i = 0; i < argc - 3; ++i) {
                pValue = PyLong_FromLong(atoi(argv[i + 3]));
                PyTuple_SetItem(pArgs, i, pValue);
            }
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                //printf("Result of call: %ld\n", PyLong_AsLong(pValue));
                windspeed=PyFloat_AsDouble(pValue);
                printf("Wind speed= %f\n", windspeed);
                Py_DECREF(pValue);       
            }
        } 
    return 0;
}

// g++ -I/usr/include/python3 -I/usr/include/python3.6m -I/home/hakim/anaconda3/  CPP_test.cpp -L/usr/lib/python3.6m/config-3.6m-x86_64-linux-gnu -lpython3.6m -o out.exe
//./out.exe Python_test Multiply 2 2
//./out.exe Python_test multiply 2 2 

//g++ -I/usr/include/python3 -I/usr/include/python3.7 -I/home/hakim/anaconda3/  CPP_test.cpp -L/usr/lib/python3.6m/config-3.7-x86_64-linux-gnu -lpython3.6m -o out.exe
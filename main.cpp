#include <iostream>
#include <vector>
#include "LaserScannerDriver.h"

int main(){
    using namespace std;
    
    //COSTRUTTORE DI DEFAULT
        cout<<"\n\n*************** TEST COSTRUTTORI ***************\n";
        LaserScannerDriver lsd0;
        cout<<"\nlsd0 state: \n";
        print_state(lsd0);

    //COSTRUTTORE CON ARGOMENTO
        LaserScannerDriver lsd1(0.5); //cambiare ris. ang. per testare anche i casi limite
        cout<<"\nlsd1 state: \n";
        print_state(lsd1);

    //NEW_SCAN
    cout<<"\n\n*************** TEST NEW_SCAN AND OPERATOR << ***************\n";
    for(int i=0; i<5; i++){
        vector<double> v1; //vettore dimensione giusta
        for(int i=0; i<lsd1.get_size_of_columns(); i++)
            v1.push_back(i);
        vector<double> v2; //vettore dimensione in eccesso
        for(int i=0; i<lsd1.get_size_of_columns()*2; i++)
            v2.push_back(i*10);
        vector<double> v3; //vettore dimensione in difetto
        for(int i=0; i<lsd1.get_size_of_columns()/2; i++)
            v3.push_back(i*100);
        lsd1.new_scan(v1);
        //cout<<lsd1<<endl; //rimuovere commento per testare operatore <<
        lsd1.new_scan(v2);
        //cout<<lsd1<<endl; //rimuovere commento per testare operatore <<
        lsd1.new_scan(v3);
        //cout<<lsd1<<endl; //rimuovere commento per testare operatore <<
    }
    cout<<"Is lsd1 full now? "<<lsd1.isFull()<<endl; //verifica se il buffer è pieno(y)

    //TEST JUST KIDDING
    cout<<"\n\n*************** TEST JK ***************\n";
    LaserScannerDriver lsd1_copy_jk = lsd1;
    cout<<"head: "<<lsd1_copy_jk.get_head()<<" tail: "<<lsd1_copy_jk.get_tail()<<endl;
    vector<double> v = lsd1_copy_jk.get_scan();
    cout<<"! GET SCAN DONE ! \n";
    cout<<"head: "<<lsd1_copy_jk.get_head()<<" tail: "<<lsd1_copy_jk.get_tail()<<endl;
    for(int i=0; i<v.size(); i++){ 
        cout<<v[i]<< "  ";
    }
    cout<<" \n";
    lsd1_copy_jk.new_scan(v);
    cout<<"! NEW SCAN DONE ! \n";
    cout<<"head: "<<lsd1_copy_jk.get_head()<<" tail: "<<lsd1_copy_jk.get_tail()<<endl;
    cout<<lsd1_copy_jk<<endl; 
    
    //GET_DISTANCE
    cout<<"\n\n*************** TEST GET_DISTANCE ***************\n";
    cout<<lsd1<<endl;
    bool tmp = true;
    for(int i=1; i<=8; i++){
        double angle;
        if(tmp)
            angle = (i*lsd1.get_ang_resolution())+2.3; //per testare arrotondamenti
        if(!tmp)
            angle = (i*lsd1.get_ang_resolution())-2.3; //per testare arrotondamenti

    lsd1.get_distance(angle);
    //cout<<"\nN^ " << i << "\tANGLE = " << angle << "\tGET_DISTANCE = " <<lsd1.get_distance(angle)<<endl;
    tmp = !tmp;
    }
    lsd1.get_distance(-0.5); //test get_distance under min
    lsd1.get_distance(181); //test get_distance over max
    
    //COPY_CONSTRUCTOR
    cout<<"\n\n*************** TEST COSTRUTTORE DI COPIA ***************\n";
    cout<<"\nlsd1 state: \n";
    print_state(lsd1);
    LaserScannerDriver lsd1_copy_constructor = lsd1;
    cout<<"\nlsd1_copy_constructor state: \n";
    print_state(lsd1_copy_constructor);

    //COPY_ASSIGNMENT
    cout<<"\n\n*************** TEST ASSEGNAMENTO DI COPIA ***************\n";
    LaserScannerDriver lsd1_copy_assignment(lsd1.get_ang_resolution());
    lsd1_copy_assignment = lsd1;
    cout<<"\nlsd1_copy_assignment state: \n";
    print_state(lsd1_copy_assignment);


    //COPY_ASSIGNMENT WRONG ANGULAR RESOLUTION
    cout<<"\n\n*************** TEST ASSEGNAMENTO DI COPIA WRONG ANG RES ***************\n";
    LaserScannerDriver lsd1_copy_assignment_w1(lsd1.get_ang_resolution() + 5);
    try{
        lsd1_copy_assignment_w1 = lsd1;
    }
    catch(...)
    {
        std::cerr << "Didn't copy, as expected" << '\n';
    }
    
    cout<<"\nlsd1_copy_assignment_w1 state: \n";
    print_state(lsd1_copy_assignment_w1);


    //CLEAR_BUFFER & GET_SCAN
    cout<<"\n\n*************** TEST CLEAR_BUFFER & GET_SCAN ***************\n";
    lsd1.clear_buffer();
    cout<<"Is lsd1 empty? "<<lsd1.isEmpty()<<endl; //expected y

    //MOVE_CONSTRUCTOR
    cout<<"\n\n*************** TEST COSTRUTTORE DI MOVE ***************\n";
    LaserScannerDriver lsd1_move_constructor = move(lsd1_copy_constructor);
    cout<<"Is lsd1_copy_constructor empty? "<<lsd1_copy_constructor.isEmpty()<<endl; //expected 1
    cout<<"\nlsd1_copy_constructor state: \n";
    print_state(lsd1_copy_constructor);
    cout<<"\nlsd1_move_constructor state: \n";
    print_state(lsd1_move_constructor);

    //MOVE_ASSIGNMENT
    cout<<"\n\n*************** TEST ASSEGNAMENTO DI MOVE ***************\n";
    LaserScannerDriver lsd1_move_assignment(lsd1_copy_assignment.get_ang_resolution());
    lsd1_move_assignment = move(lsd1_copy_assignment);
    cout<<"Is lsd1_copy_assignment empty? "<<lsd1_copy_assignment.isEmpty()<<endl; //expected 1
    cout<<"\nlsd1_copy_assignment state: \n";
    print_state(lsd1_copy_assignment);
    cout<<"\nlsd1_move_assignment state: \n";
    print_state(lsd1_move_assignment);

    cout<<"\n\n****************************** PROGRAMMA TERMINATO ******************************\n";
    return 0;
}

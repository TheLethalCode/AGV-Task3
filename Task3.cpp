#include<bits/stdc++.h>
#include<eigen3/Eigen/Dense>

#define fi first
#define se second
#define ma make_pair
#define pb push_back

using namespace std;
using namespace Eigen;

int main(int argc, char const *argv[]) {

  fstream file(argv[1]);
  if(!file){
    cout<<"Unable to open file";        //Opening the file
    return -1;
  }

  vector< pair<float,float> > Data;
  while(!file.eof())  {
    float x,y;
    file>>x;
    file.ignore();           // Reading the data from the file and storing it in a vector
    file>>y;
    Data.pb(ma(x,y));
  }

  float time_step;
  if(argc<=2){
    cout<<"Enter the time_step"<<endl;
    cin>>time_step;
  }                             // Getting the time step
  else{
    stringstream temp(argv[2]);
    temp>>time_step;
  }

  float Px,Py,Vx,Vy,time1=0;
  cout<<"Enter initial coordinates :- ";
  cin>>Px>>Py;                                //Getting the initial dat
  cout<<"Enter initial x-velocity and y-velocity :-";
  cin>>Vx>>Vy;

  MatrixXd P(2,2),te_P(2,2),X(2,2),te_X(2,2),R(2,2),F(2,2);
  te_X<<Px,Py,Vx,Vy;
  te_P<<1000,0,0,1000;                    // Deg=fining all the necessary matrices
  R<<0.1,0,0,0.1;
  F<<1,time_step,0,1;

  cout<<"\nTIME STEP = "<<time_step<<endl<<endl;
  cout<<endl<<"INITIALLY \n\nPOSITION = ( "<<Px<<" , "<<Py<<" )"<<endl;
  cout<<"VELOCITY = "<<Vx<<"i + "<<Vy<<"j"<<endl;
  cout<<"UNCERTAINITY MATRIX :- \n"<<te_P<<endl;
  cout<<"============================================================================="<<endl;

  for(vector< pair<float,float> > :: iterator it = Data.begin();it!=Data.end();it++){
    time1+=time_step;
    Vx = it->fi - Px;
    Vx/=time_step;             // Calculating the velocity based on the sensor readings
    Vy = it->se - Py;
    Vy/=time_step;
    Px = it->fi;
    Py = it->se;

    MatrixXd Z(2,2);               // Defining the matrix for the sensor readings
    Z<<Px,Py,Vx,Vy;

    X=F*te_X;                // Calculating the new state and uncertainity from prior knowledge
    P= F*(te_P*(F.transpose()));

    MatrixXd K(2,2),PRi(2,2);
    PRi = P + R;              // Calculating K
    K = P*(PRi.inverse());

    X = X + K*(Z-X);         // Combining both the data
    P = P - K*P;

    te_X = X;             // The prior state acts as the source  for the next state
    te_P = P;

    // Printing them
    cout<<fixed<<setprecision(4)<<"AT TIME = "<<time1<<" \n\nPOSITION = ( "<<X(0,0)<<" , "<<X(0,1)<<" )"<<endl;
    cout<<fixed<<setprecision(4)<<"VELOCITY = "<<X(1,0)<<"i + "<<X(1,1)<<"j"<<endl;
    cout<<fixed<<setprecision(4)<<"UNCERTAINITY MATRIX :- \n"<<P<<endl<<endl;
    cout<<"============================================================================="<<endl;

  }

  return 0;
}

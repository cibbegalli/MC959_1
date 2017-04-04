#include <iostream>
#include <vector>
#include <iomanip>

using namespace std;

double calc_vel(vector<bool> detect) {
    double maxDetectionDist = 0.5;
    double noDetectionDist = 0.2;
    
    double wL[8] = {-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6};
    double wR[8] = {-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2};

    double dist[3] = {0.3, 0.4, 0.5};
    
    for(int i=0; i<3; i++) {

        double vL = 2.0;
        double vR = 2.0;
        
        for(int j=0; j<8; j++) {
            if(detect[j]) {
                double factor = 1-((dist[i]-maxDetectionDist)/(noDetectionDist-maxDetectionDist));
                vL += factor*wL[j];
                vR += factor*wR[j];
            }
        }
        cout << setprecision(1) << fixed;
        cout << " {" << vL << "," << vR << "} ";
    }
}

void comb(vector<bool> &detect, int k) {
    if(k==8) {
        cout << "[ ";
        for(int i=0; i<8; i++) {
            cout << detect[i] << " ";
        }
        cout << "] ";
        calc_vel(detect);
        cout << "\n";
    } else {
        detect[k] = false;
        comb(detect, k+1);

        detect[k] = true;
        comb(detect, k+1);
    }
}

int main(int argc, char *argv[])
{
    vector<bool> detect(8, false);
    comb(detect, 0);

    return 0;
}

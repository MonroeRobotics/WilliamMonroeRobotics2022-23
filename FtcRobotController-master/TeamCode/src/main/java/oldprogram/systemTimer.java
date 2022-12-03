package oldprogram;

public class systemTimer {
    long start;
    double ms;

    public void SystemTimer(){
    }

    public void startTimer(double seconds){
        start = System.currentTimeMillis();
        ms = seconds * 100;
    }

    public boolean check(){
        return System.currentTimeMillis() - start <= ms;
    }
}

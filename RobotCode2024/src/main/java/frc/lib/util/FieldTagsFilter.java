package frc.lib.util;

public class FieldTagsFilter {
    boolean _redOrBlue;
    int[] ids;


    public FieldTagsFilter (boolean redOrBlue){
        if (redOrBlue) {
            blue();
        }
        else
        {
            red();
        }
    }
    public boolean isRelavent(int id){
        for (int i : ids) {
            return i == id;
        }
        return false;
    }
    void red(){
        ids = new int[]{
            3,4,5,
            11,12,13
        };
        
    }
    void blue(){
        ids = new int[]{
        1,2,6,
        14,15,16
        };
        
    }

    public int getAmp(){
        return ids[2];
      }
    
    public int getRightSource(){
        return ids[0];
    }

    public int getLeftSource(){
        return ids[1];
    }
    
    
}

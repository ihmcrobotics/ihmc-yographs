package us.ihmc.yoGraphs;

public interface GraphConfigurationChangeListener
{
   void notifyOfGraphTypeChange();

   void notifyOfBaselineChange();

   void notifyOfScaleChange();

   void notifyOfDisplayChange();
}

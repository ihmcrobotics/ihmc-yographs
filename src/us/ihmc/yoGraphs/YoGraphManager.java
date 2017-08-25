package us.ihmc.yoGraphs;

import us.ihmc.yoGraphs.graphInterfaces.GraphIndicesHolder;
import us.ihmc.yoGraphs.graphInterfaces.SelectedVariableHolder;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.dataBuffer.DataBufferChangeListener;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntry;
import us.ihmc.yoVariables.dataBuffer.IndexChangedListener;

import java.util.ArrayList;

/**
 * Wrapper for YoGraphs to handle the graph indices and data buffer.
 *
 * @author Carson Wilber
 * @since 2017-08-25
 */
public class YoGraphManager implements GraphIndicesHolder, DataBufferChangeListener
{
   private ArrayList<YoGraph> graphs = new ArrayList<>();

   private DataBuffer dataBuffer;

   private int leftPlotIndex;

   private int rightPlotIndex;

   private int doTick = 0;

   private int doIndex = -1;

   private SelectedVariableHolder selectedVariableHolder;

   private ArrayList<IndexChangedListener> indexChangeListeners = new ArrayList<>();

   public YoGraphManager(SelectedVariableHolder holder, DataBuffer buffer)
   {
      this.selectedVariableHolder = holder;

      this.dataBuffer = buffer;

      this.leftPlotIndex = 0;

      this.rightPlotIndex = getMaxIndex();
   }

   /**
    * Calls for an immediate change of index to the in point in the data buffer.
    */
   public void goToInPointNow()
   {
      dataBuffer.gotoInPoint();
   }

   /**
    * Calls for an immediate change of index to the out point in the data buffer.
    */
   public void goToOutPointNow()
   {
      dataBuffer.gotoOutPoint();
   }

   /**
    * Calls for an immediate tick in the data buffer.
    *
    * @param n integer number of ticks to tick
    * @return boolean if the data buffer rolled over to the in point
    */
   public boolean tick(int n)
   {
      return dataBuffer.tick(n);
   }

   @Override public void tickLater(int n)
   {
      if (dataBuffer.isKeyPointModeToggled())
      {
         if (n > 0)
         {
            setIndexLater(dataBuffer.getNextTime());
         }
         else
         {
            setIndexLater(dataBuffer.getPreviousTime());
         }
      }
      else
      {
         this.doTick = n;
      }
   }

   /**
    * Calls for a delayed index change in the data buffer.
    *
    * <p>Change occurs next time {@link #allowTickUpdatesNow()} is called.
    *
    * @param idx integer index to change to
    */
   @Override public void setIndexLater(int idx)
   {
      this.doIndex = idx;
   }

   @Override public int getInPoint()
   {
      return dataBuffer.getInPoint();
   }

   @Override public int getOutPoint()
   {
      return dataBuffer.getOutPoint();
   }

   @Override public int getIndex()
   {
      return dataBuffer.getIndex();
   }

   @Override public boolean isIndexAtOutPoint()
   {
      return (getIndex() == getOutPoint());
   }

   @Override public void attachIndexChangeListener(IndexChangedListener listener)
   {
      this.indexChangeListeners.add(listener);
   }

   @Override public void detachIndexChangeListener(IndexChangedListener listener)
   {
      this.indexChangeListeners.remove(listener);
   }

   @Override public void notifyIndexChangeListeners()
   {
      for (IndexChangedListener listener : this.indexChangeListeners)
      {
         listener.notifyOfIndexChange(getIndex());
      }
   }

   @Override public int getMaxIndex()
   {
      return dataBuffer.getBufferSize() - 1;
   }

   @Override public int getLeftPlotIndex()
   {
      return this.leftPlotIndex;
   }

   @Override public int getRightPlotIndex()
   {
      return this.rightPlotIndex;
   }

   @Override public void setLeftPlotIndex(int idx)
   {
      this.leftPlotIndex = idx;
   }

   @Override public void setRightPlotIndex(int idx)
   {
      this.rightPlotIndex = idx;
   }

   @Override public ArrayList<Integer> getKeyPoints()
   {
      return dataBuffer.getKeyPoints();
   }

   @Override public void notifyOfBufferChange()
   {
      rightPlotIndex = Math.min(rightPlotIndex, getMaxIndex());
   }

   @Override public void notifyOfManualEndChange(int inPoint, int outPoint)
   {
      // do nothing; does not affect manager
   }

   public void zoomFullView()
   {
      leftPlotIndex = 0;

      rightPlotIndex = getMaxIndex();
   }

   public void zoomIn()
   {
      zoomIn(2);
   }

   public void zoomIn(int factor)
   {
      int index = getIndex();

      int oldLength = rightPlotIndex - leftPlotIndex;

      int newLength = oldLength / factor;

      if (newLength < 4)
      {
         return;
      }

      leftPlotIndex = index - newLength / 2;

      rightPlotIndex = leftPlotIndex + newLength;

      if (leftPlotIndex < 0)
      {
         leftPlotIndex = 0;

         rightPlotIndex = leftPlotIndex + newLength;

         if (rightPlotIndex > getMaxIndex())
         {
            rightPlotIndex = getMaxIndex();
         }
      }
      else if (rightPlotIndex > getMaxIndex())
      {
         rightPlotIndex = getMaxIndex();

         leftPlotIndex = rightPlotIndex - newLength;

         if (leftPlotIndex < 0)
         {
            leftPlotIndex = 0;
         }
      }
   }

   public void zoomOut()
   {
      zoomOut(2);
   }

   public void zoomOut(int factor)
   {
      int index = this.getIndex();

      int oldLength = rightPlotIndex - leftPlotIndex;

      int newLength = oldLength * factor;

      leftPlotIndex = index - newLength / 2;

      rightPlotIndex = leftPlotIndex + newLength;

      if (leftPlotIndex < 0)
      {
         leftPlotIndex = 0;

         rightPlotIndex = leftPlotIndex + newLength;

         if (rightPlotIndex > getMaxIndex())
         {
            rightPlotIndex = getMaxIndex();
         }
      }
      else if (rightPlotIndex > getMaxIndex())
      {
         rightPlotIndex = getMaxIndex();

         leftPlotIndex = rightPlotIndex - newLength;

         if (leftPlotIndex < 0)
         {
            leftPlotIndex = 0;
         }
      }
   }

   public void recenter()
   {
      zoomIn(1);
   }

   public boolean allowTickUpdatesNow()
   {
      boolean ret = false;

      if (this.doTick != 0)
      {
         dataBuffer.tick(doTick);

         ret = true;

         doTick = 0;
      }

      if (this.doIndex != -1)
      {
         dataBuffer.setIndex(this.doIndex);

         this.doIndex = -1;

         ret = true;
      }

      return ret;
   }

   public void setupGraph(String fvar, String... vars)
   {
      if (fvar == null || dataBuffer.getEntry(fvar) == null)
      {
         return;
      }

      YoGraph g = new YoGraph(this, selectedVariableHolder, dataBuffer, dataBuffer);

      g.addVariable(dataBuffer.getEntry(fvar));

      for (String var : vars)
      {
         DataBufferEntry entry = dataBuffer.getEntry(var);

         if (entry != null)
         {
            g.addVariable(entry);
         }
      }

      this.add(g);
   }

   public void setupGraph(GraphConfiguration config, String fvar, String... vars)
   {
      if (fvar == null || dataBuffer.getEntry(fvar) == null)
      {
         return;
      }

      YoGraph g = new YoGraph(this, selectedVariableHolder, dataBuffer, dataBuffer);

      g.addVariable(dataBuffer.getEntry(fvar));

      for (String var : vars)
      {
         DataBufferEntry entry = dataBuffer.getEntry(var);

         if (entry != null)
         {
            g.addVariable(entry);
         }
      }

      if (config != null)
      {
         g.setGraphConfiguration(config);
      }

      this.add(g);
   }

   public void add(YoGraph graph)
   {
      this.graphs.add(graph);

      dataBuffer.attachIndexChangedListener(graph);
   }

   public void remove(YoGraph graph)
   {
      dataBuffer.detachIndexChangedListener(graph);

      this.graphs.remove(graph);
   }

   public void clear()
   {
      for (YoGraph graph : this.graphs) {
         this.remove(graph);
      }
   }
}

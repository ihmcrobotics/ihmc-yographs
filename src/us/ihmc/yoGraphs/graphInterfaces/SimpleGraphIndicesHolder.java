package us.ihmc.yoGraphs.graphInterfaces;

import us.ihmc.yoVariables.dataBuffer.IndexChangedListener;

import java.util.ArrayList;

public class SimpleGraphIndicesHolder implements GraphIndicesHolder
   {
      int index = 0;
      int dataBufferSize;

      private ArrayList<IndexChangedListener> changeListeners = new ArrayList<>();
      
      public SimpleGraphIndicesHolder(int dataBufferSize)
      {
         this.dataBufferSize = dataBufferSize;
      }

      @Override
      public void tickLater(int i)
      {
         index += i;
      }

      @Override
      public void setRightPlotIndex(int newRightIndex)
      {
      }

      @Override
      public void setLeftPlotIndex(int newLeftIndex)
      {
      }

      @Override
      public void setIndexLater(int newIndex)
      {
      }

      @Override
      public boolean isIndexAtOutPoint()
      {
         return getIndex() == getOutPoint();
      }

      @Override public void attachIndexChangeListener(IndexChangedListener listener)
      {
         this.changeListeners.add(listener);
      }

      @Override public void detachIndexChangeListener(IndexChangedListener listener)
      {
         this.changeListeners.remove(listener);
      }

      @Override public void notifyIndexChangeListeners()
      {
         for (IndexChangedListener listener : this.changeListeners) {
            listener.notifyOfIndexChange(index);
         }
      }

      @Override
      public int getRightPlotIndex()
      {
         return dataBufferSize;
      }

      @Override
      public int getOutPoint()
      {
         return dataBufferSize;
      }

      @Override
      public int getMaxIndex()
      {
         return dataBufferSize;
      }

      @Override
      public int getLeftPlotIndex()
      {
         return 0;
      }

      @Override
      public ArrayList<Integer> getKeyPoints()
      {
         return null;
      }

      @Override
      public int getIndex()
      {
         return index;
      }

      @Override
      public int getInPoint()
      {
         return 0;
      }
   }
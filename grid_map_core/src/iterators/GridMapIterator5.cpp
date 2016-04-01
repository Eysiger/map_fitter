/*
 * GridMapIterator5.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/GridMapIterator5.hpp"
#include "grid_map_core/GridMapMath.hpp"

namespace grid_map {

GridMapIterator5::GridMapIterator5(const grid_map::GridMap& gridMap, const int every)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  linearSize_ = size_.prod();
  linearIndex_ = 0;
  isPastEnd_ = false;
  every_ = every;
}

GridMapIterator5::GridMapIterator5(const GridMapIterator5* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  linearSize_ = other->linearSize_;
  linearIndex_ = other->linearIndex_;
  isPastEnd_ = other->isPastEnd_;
}

GridMapIterator5& GridMapIterator5::operator =(const GridMapIterator5& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  linearSize_ = other.linearSize_;
  linearIndex_ = other.linearIndex_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool GridMapIterator5::operator !=(const GridMapIterator5& other) const
{
  return linearIndex_ != other.linearIndex_;
}

const Index GridMapIterator5::operator *() const
{
  return getIndexFromLinearIndex(linearIndex_, size_);
}

const size_t& GridMapIterator5::getLinearIndex() const
{
  return linearIndex_;
}

const Index GridMapIterator5::getUnwrappedIndex() const
{
  return getIndexFromBufferIndex(*(*this), size_, startIndex_);
}

GridMapIterator5& GridMapIterator5::operator ++()
{
  size_t newIndex;
  if (((int)(linearIndex_+every_)/size_(0)) % every_ == 0) {
	newIndex = linearIndex_ + every_;
  }
  else {
	newIndex = linearIndex_ + (every_-1)*size_(0);
  }
  if (newIndex < linearSize_) {
    linearIndex_ = newIndex;
  } else {
    isPastEnd_ = true;
  }
  return *this;
}

GridMapIterator5 GridMapIterator5::end() const
{
  GridMapIterator5 res(this);
  res.linearIndex_ = linearSize_ - 1;
  return res;
}

bool GridMapIterator5::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace grid_map */

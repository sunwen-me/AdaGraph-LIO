#ifndef LI2SUP_KISS_MATCHER_RELOCALIZER_H_
#define LI2SUP_KISS_MATCHER_RELOCALIZER_H_

#include "basic/alias.h"
#include "basic/Manifold.h"

namespace LI2Sup {

class KISSMatcherRelocalizer {
public:
  bool estimate(const BASIC::CloudPtr& source,
                const BASIC::CloudPtr& target,
                BASIC::SE3& target_source) const;
};

}  // namespace LI2Sup

#endif

#include "twist_to_manual_control_alg.h"

TwistToManualControlAlgorithm::TwistToManualControlAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

TwistToManualControlAlgorithm::~TwistToManualControlAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void TwistToManualControlAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// TwistToManualControlAlgorithm Public API

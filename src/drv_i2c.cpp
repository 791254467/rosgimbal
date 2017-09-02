#include "drv_i2c.h"

//global i2c ptrs used by the event interrupts
I2C* I2CDev_1Ptr;
I2C* I2CDev_2Ptr;

I2C::I2C(I2C_TypeDef *I2C) {
  dev = I2C;
  //enable peripheral clocks as we need them
  if (dev == I2C1)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  }
  else if (dev == I2C2)
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  }
  init();
}

void I2C::init(void) {
  GPIO_InitTypeDef gpio_init_struct;
  I2C_InitTypeDef i2c_init_struct;
  NVIC_InitTypeDef nvic_init_struct;
  uint8_t er_irq;
  uint8_t ev_irq;

  if (dev == I2C1)
  {
    //configure the gpio pins
    GPIO_PinAFConfig(I2C1_GPIO, I2C1_SCL_PIN_SOURCE, GPIO_AF_I2C1);
    GPIO_PinAFConfig(I2C1_GPIO, I2C1_SDA_PIN_SOURCE, GPIO_AF_I2C1);
    sda_.init(I2C1_GPIO, I2C1_SDA_PIN, GPIO::PERIPH_IN_OUT);
    scl_.init(I2C1_GPIO, I2C1_SCL_PIN, GPIO::PERIPH_IN_OUT);
    er_irq = I2C1_ER_IRQn;
    ev_irq = I2C1_EV_IRQn;
    I2CDev_1Ptr = this;
  }
  else if (dev == I2C2)
  {
    //configure the gpio pins
    GPIO_PinAFConfig(I2C2_GPIO, I2C2_SCL_PIN_SOURCE, GPIO_AF_I2C2);
    GPIO_PinAFConfig(I2C2_GPIO, I2C2_SDA_PIN_SOURCE, GPIO_AF_I2C2);
    sda_.init(I2C2_GPIO, I2C2_SDA_PIN, GPIO::PERIPH_IN_OUT);
    scl_.init(I2C2_GPIO, I2C2_SCL_PIN, GPIO::PERIPH_IN_OUT);
    er_irq = I2C2_ER_IRQn;
    ev_irq = I2C2_EV_IRQn;
    I2CDev_2Ptr = this;
  }

  unstick(); //unstick will properly initialize pins


  if (dev == I2C1 || dev == I2C2)
  {
    //initialze the i2c itself
    I2C_DeInit(dev);

    I2C_StructInit(&i2c_init_struct);
    i2c_init_struct.I2C_ClockSpeed = 400000;
    i2c_init_struct.I2C_Mode = I2C_Mode_I2C;
    i2c_init_struct.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c_init_struct.I2C_OwnAddress1 = 0; //The first device address
    i2c_init_struct.I2C_Ack = I2C_Ack_Disable;
    i2c_init_struct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(dev, &i2c_init_struct);
    I2C_Cmd(dev, ENABLE);

    //initialize the interrupts
    nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 2;
    nvic_init_struct.NVIC_IRQChannelSubPriority = 1;
    nvic_init_struct.NVIC_IRQChannelCmd	= ENABLE;

    nvic_init_struct.NVIC_IRQChannel = er_irq;
    NVIC_Init(&nvic_init_struct);

    //I2C Event Interrupt
    nvic_init_struct.NVIC_IRQChannel = ev_irq;
    NVIC_Init(&nvic_init_struct);

  }
}

void I2C::unstick()
{
  scl_.set_mode(GPIO::OUTPUT);
  sda_.set_mode(GPIO::OUTPUT);

  scl_.write(GPIO::HIGH);
  sda_.write(GPIO::HIGH);

  for (int i = 0; i < 8; ++i)
  {
    delayMicroseconds(3);
    scl_.toggle();
  }

  sda_.write(GPIO::LOW);
  delayMicroseconds(3);
  scl_.write(GPIO::LOW);
  delayMicroseconds(3);

  scl_.write(GPIO::HIGH);
  delayMicroseconds(3);
  sda_.write(GPIO::HIGH);
  delayMicroseconds(3);

  scl_.set_mode(GPIO::PERIPH_IN_OUT);
  sda_.set_mode(GPIO::PERIPH_IN_OUT);
}

bool I2C::read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
  addr_ = addr << 1;
  reg_ = reg;
  len_ = len;
  data_buffer_ = data;
  index_ = 0;
  reading_ = true;
  busy_ = true;
  error_ = false;
  subaddress_sent_ = (reg_ && 0xFF);

  uint32_t timeout = I2C_TIMEOUT_US;

  //dont do anything if the device is restarting
  if (!(dev->CR2 & I2C_IT_EVT))
  {
    //check if we're supposed to send a start
    if (!(dev->CR1 & I2C_CR1_START))
    {
      // Wait for any stop to finish
      while (dev->CR1 & I2C_CR1_STOP && --timeout > 0);
      if (timeout == 0)
      {
        handle_hardware_failure();
        return false;
      }
      // Start the new job
      I2C_GenerateSTART(dev, ENABLE);
    }
    // Allow the interrupts to fire off again
    I2C_ITConfig(dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
  }

  //probably want to remove for fully interupt-based
  timeout = I2C_TIMEOUT_US;
  while(busy_ && --timeout > 0);
  if (timeout == 0)
  {
    handle_hardware_failure();
    return false;
  }

  return true;
}

bool I2C::write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
  addr_ = addr << 1;
  reg_ = reg;
  len_ = len;
  data_buffer_ 	 = data;
  index_ = 0;
  reading_ = false;
  busy_ = true;
  error_ = false;
  subaddress_sent_ = (reg_ && 0xFF);

  uint32_t timeout = I2C_TIMEOUT_US;

  //dont do anything if the device is restarting
  if (!(dev->CR2 & I2C_IT_EVT))
  {
    //check if we're supposed to send a start
    if (!(dev->CR1 & I2C_CR1_START))
    {
      // Wait for any stop to finish
      while (dev->CR1 & I2C_CR1_STOP && --timeout > 0);
      if (timeout == 0)
      {
        handle_hardware_failure();
        return false;
      }
      // Start the new job
      I2C_GenerateSTART(dev, ENABLE);
    }
    // Allow the interrupts to fire off again
    I2C_ITConfig(dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
  }

  //probably want to remove for fully interupt-based
  timeout = I2C_TIMEOUT_US;
  while(busy_ && --timeout > 0);
  if (timeout == 0)
  {
    handle_hardware_failure();
    return false;
  }

  return true;
}

bool I2C::read(uint8_t addr, uint8_t reg, uint8_t *data) {
  return read(addr, reg, 1, data);
}

bool I2C::write(uint8_t addr, uint8_t reg, uint8_t data) {
  return write(addr, reg, 1, &data);
}

void I2C::handle_hardware_failure() {
  error_count_++;
  init(); //unstick and reinitialize the hardware
}

void I2C::handle_error(){
  //grab this device's status registers
  volatile uint32_t sr1 = dev->SR1;
  volatile uint32_t sr2 = dev->SR2;

  // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
  if (sr1 & (I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR))
  {
    I2C_ITConfig(dev, I2C_IT_BUF, DISABLE);
    if (!(sr1 & I2C_SR1_ARLO) && !(dev->CR1 & I2C_CR1_STOP)) // If we dont have an ARLO error, ensure sending of a stop
    {
      if (dev->CR1 & I2C_CR1_START) // We are currently trying to send a start, this is very bad as start,stop will hang the peripheral
      {
        while (dev->CR1 & I2C_CR1_START); // Wait for any start to finish sending
        I2C_GenerateSTOP(dev, ENABLE); // Send stop to finalise bus transaction
        while (dev->CR1 & I2C_CR1_STOP); // Wait for stop to finish sending
        init();												// Reset and configure the hardware
      }
      else
      {
        I2C_GenerateSTOP(dev, ENABLE); // Stop to free up the bus
        I2C_ITConfig(dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE); // Disable EVT and ERR interrupts while bus inactive. They'll be reenabled
      }
    }
  }

  //reset errors
  dev->SR1 &= ~(I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);
  busy_ = false;
}

void I2C::handle_event() {
  //grab the bottom 8 bits of this device's status register
  uint8_t sr1 = dev->SR1;


  if (sr1 & I2C_SR1_SB) // EV5 (in ref manual) - Start just sent
  {
    dev->CR1 &= ~I2C_CR1_POS; // Reset the POS bit so ACK/NACK applied to the current byte
    I2C_AcknowledgeConfig(dev, ENABLE);	// Make sure ACK is on
    index_ = 0;
    if (reading_ && (subaddress_sent_ || reg_ == 0xFF))
    {
      subaddress_sent_ = true;
      if (len_ == 2)
      {
        dev->CR1 |= I2C_CR1_POS; // if doing a 2-byte read, set NACK to apply to the final byte
      }
      I2C_Send7bitAddress(dev, addr_, I2C_Direction_Receiver);	// start a read
    }
    else // We are either writing or the subaddress hasn't been sent and we need to write it
    {
      I2C_Send7bitAddress(dev, addr_, I2C_Direction_Transmitter); //start a write
      if (!subaddress_sent_)
      {
        index_ = -1;  // Send the subaddress
      }
    }
  }

  else if (sr1 & I2C_SR1_ADDR) // EV6 - Address just sent
  {
    __DMB();
    if (reading_ && subaddress_sent_)
    {
      if (len_ == 1) // EV6_1, set the stop
      {
        I2C_AcknowledgeConfig(dev, DISABLE);
        __DMB();
        (void)dev->SR2; // read SR2 to clear the ADDR bit
        I2C_GenerateSTOP(dev, ENABLE);
        I2C_ITConfig(dev, I2C_IT_BUF, ENABLE);  // enable EVT_7
      }
      else // receiving greater than one byte
      {
        (void)dev->SR2; 		// read SR2 to clear the ADDR bit
        __DMB();
        if (len_ == 2)
        {
          I2C_AcknowledgeConfig(dev, DISABLE);
          I2C_ITConfig(dev, I2C_IT_BUF, DISABLE); // Disable TXE to allow the buffer to fill
        }
        else if (len_ == 3) // Receiving three bytes
        {
          I2C_ITConfig(dev, I2C_IT_BUF, DISABLE);  // Make sure RXNE disabled so we get a BTF in two bytes time
        }
        else  // Receiving greater than three bytes
        {
          I2C_ITConfig(dev, I2C_IT_BUF, ENABLE);  //enables EVT_7
        }
      }
    }
    else // sending subaddress, or transmitting
    {
      (void)dev->SR2; // read SR2 to clear the ADDR bit
      __DMB();
      I2C_ITConfig(dev, I2C_IT_BUF, ENABLE);  //enables EVT_7
    }
  }

  else if (sr1 & I2C_SR1_BTF) // EV7_2, EV7_3, EV8_2 - Byte Transfer Finished
  {
    if (reading_ && subaddress_sent_) // EV7_2, EV7_3
    {
      if (len_ > 2) // EV7_2
      {
        I2C_AcknowledgeConfig(dev, DISABLE); // Turn off ACK
        data_buffer_[index_++] = I2C_ReceiveData(dev);	// Read second to last byte
        I2C_GenerateSTOP(dev, ENABLE); // Program the Stop
        data_buffer_[index_++] = I2C_ReceiveData(dev); // Read last byte
        I2C_ITConfig(dev, I2C_IT_BUF, ENABLE);	// Enable for final EV7
      }
      else // EV7_3
      {
        I2C_GenerateSTOP(dev, ENABLE); // Program the Stop
        data_buffer_[index_++] = I2C_ReceiveData(dev);	// Read data
        data_buffer_[index_++] = I2C_ReceiveData(dev);	// Read data
        index_++;
      }
    }
    else // EV8_2, which may be due to a subaddress sent or a write completion
    {
      if (subaddress_sent_ || (!reading_))
      {
        I2C_GenerateSTOP(dev, ENABLE);
        index_++; // Indicate that the job is finished
      }
      else // we just wrote the subaddress
      {
        I2C_GenerateSTART(dev, ENABLE); // We need a repeated start here
        subaddress_sent_ = true;
      }
    }
    while (dev->CR1 & I2C_CR1_START); // Wait for the start to clear, to avoid BTF
  }

  else if (sr1 & I2C_SR1_RXNE)	// Byte received - EV7
  {
    data_buffer_[index_++] = I2C_ReceiveData(dev);
    if (len_ == (index_ + 3))
      I2C_ITConfig(dev, I2C_IT_BUF, DISABLE); // Disable RxNE
    if (len_ == index_)	// We have completed a final EV7
      index_++; // To show job is complete
  }

  else if (sr1 & I2C_SR1_TXE) // Byte transmitted - EV8/EV8_1
  {
    if (index_ != -1) // We've already sent subaddress
    {
      I2C_SendData(dev, data_buffer_[index_++]);
      if (len_ == index_) // We have sent all the data
        I2C_ITConfig(dev, I2C_IT_BUF, DISABLE); // Disable TXE to allow the buffer to flush
    }
    else // we need to send the subaddress
    {
      index_++;
      I2C_SendData(dev, reg_);	// Send the subaddress
      if (reading_ || !len_) // If receiving or sending 0 bytes, flush now
        I2C_ITConfig(dev, I2C_IT_BUF, DISABLE);	// Disable TXE to allow the buffer to flush
    }
  }

  if (index_ == len_ + 1) // We have completed the current jobs
  {
    subaddress_sent_ = false; // Reset this here
    I2C_ITConfig(dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE); // Disable EVT and ERR interrupts while bus inactive
    busy_ = false;
  }
}


// C-based IRQ functions (defined in the STD lib somewhere
extern "C" void I2C1_ER_IRQHandler(void) {
  I2CDev_1Ptr->handle_error();
}

extern "C" void I2C1_EV_IRQHandler(void) {
  I2CDev_1Ptr->handle_event();
}

extern "C" void I2C2_ER_IRQHandler(void) {
  I2CDev_2Ptr->handle_error();
}

extern "C" void I2C2_EV_IRQHandler(void) {
  I2CDev_2Ptr->handle_event();
}

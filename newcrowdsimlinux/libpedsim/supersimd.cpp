void Ped::Model::thisIsAFunction(){
  std::vector<Ped::Tagent*> tagent = getAgents();
  unsinged int tagent_size= tagent.size();
  
  unsigned int vector_size = tagent_size + 4 - (tagent_size % 4);
  float destinationGetX[vector_size] __attribute__((aligned (16)));
  float destinationGetY[vector_size] __attribute__((aligned (16)));
  float x[vector_size] __attribute__((aligned (16)));
  float y[vector_size] __attribute__((aligned (16)));
  
  #pragma omp parallel for 
  for(int i=0; i < tagent_size; i++){
    destinationGetX[i] = tagent[i]->destination == NULL ? 0 : tagent[i]->destination->getX();
    destinationGetY[i] = tagent[i]->destination == NULL ? 0 : tagent[i]->destination->getY();
    x[i] = tagent[i]->getX();
    y[i] = tagent[i]->getY();
  }
  
  //float diffX[vector_size]__attribute__((aligned (16)));
  //float diffY[vector_size]__attribute__((aligned (16)));
  
  int32_t agentReachedDestination[vector_size] __attribute__((aligned (16)));

  //#pragma omp parallel for 
  for(int i=0; i < vector_size; i+=4){
    // compute if agent reached its current destination
    __m128 pSrcX = _mm_load_ps(x + index);
    __m128 pSrcY = _mm_load_ps(y + index);
    __m128 pSrcDestx= _mm_load_ps(destinationGetX + index);
    __m128 pSrcDesty= _mm_load_ps(destinationGetY + index);
    __m128 diffX = _mm_sub_ps(pSrcDestx,pSrcX);
    __m128 diffY = _mm_sub_ps(pSrcDesty,pSrcY);
    //__m128 diffX = _mm_add_ps((__m128*)(destinationGetX+i),(__m128*)(x+i));
    //__m128 diffY = _mm_add_ps((__m128*)(destinationGetY+i),(__m128*)(y+i));
    //double diffX = destination->getx() - x;
    //double diffY = destination->gety() - y;
    __m128 mul1 = _mm_mul_ps(diffX, diffX);
    __m128 mul2 = _mm_mul_ps(diffY, diffY);
    __m128 mulsum = _mm_add_ps(mul1, mul2);
    __m128 length = _mm_sqrt_ps(mulsum); 
    
    //double length = sqrt(diffX * diffX + diffY * diffY);
    __m128i cmplt = _mm_cmplt_ps(a,b)
    
    _mm_store_si128(agentReachedDestination+i, cmplt);
      //agentReachedDestination = length < destination->getr();
  }
  
  
  #pragma omp parallel for 
  for(int i=0; i < tagent_size; i+=1){
  if ((agentReachedDestination[i] || tagent[i]->destination == NULL) && !tagent[i]->waypoints.empty()) {
    // Case 1: agent has reached destination (or has no current destination);
    // get next destination if available
    destination = tagent[i]->waypoints.front(); tagent[i]->waypoints.pop_front(); // pop
    
  }
  else {
    // else else else else else else else else else else else else else else else else else else else 
    // Case 2: agent has not yet reached destination, continue to move towards
    // current destination
    // destination == destination (y)
  }
     
  float notRoundedDesiredPositionXNeedToBeRoundedBeforeSettingDesiredPositionX[vector_size]__attribute__((aligned (16)));
  float notRoundedDesiredPositionYNeedToBeRoundedBeforeSettingDesiredPositionY[vector_size]__attribute__((aligned (16))    // compute if agent reached its current destination
    __m128 pSrcX = _mm_load_ps(x + index);
    __m128 pSrcY = _mm_load_ps(y + index);
    __m128 pSrcDestx= _mm_load_ps(destinationGetX + index);
    __m128 pSrcDesty= _mm_load_ps(destinationGetY + index);
    __m128 diffX = _mm_sub_ps(pSrcDestx,pSrcX);
    __m128 diffY = _mm_sub_ps(pSrcDesty,pSrcY);
    //__m128 diffX = _mm_add_ps((__m128*)(destinationGetX+i),(__m128*)(x+i));
    //__m128 diffY = _mm_add_ps((__m128*)(destinationGetY+i),(__m128*)(y+i));
    __m128 mul1 = _mm_mul_ps(diffX, diffX);
    __m128 mul2 = _mm_mul_ps(diffY, diffY);
    __m128 mulsum = _mm_add_ps(mul1, mul2);
    __m128 length = _mm_sqrt_ps(mulsum); 

    __m128 divXLength = _mm_div_ps(diffX, length);    
    __m128 divYLength = _mm_div_ps(diffY, length);
    __m128 resultDesiredX = _mm_add_ps(divXLength, pSrcX); 
    __m128 resultDesiredY = _mm_add_ps(divYLength, pSrcY); 
    _mm_store_ps(notRoundedDesiredPositionXNeedToBeRoundedBeforeSettingDesiredPositionX,resultDesiredX);       _mm_store_ps(notRoundedDesiredPositionYNeedToBeRoundedBeforeSettingDesiredPositionY,resultDesiredY);
  }

#pragma omp parallel for 
for(int i=0; i < tagent_size; i+=1){
  if (Tagent[i]->destination != NULL) {
    tagent[i]->desiredPositionX = round(notRoundedDesiredPositionXNeedToBeRoundedBeforeSettingDesiredPositionX);
    tagent[i]->desiredPositionY = round(notRoundedDesiredPositionYNeedToBeRoundedBeforeSettingDesiredPositionY);
  }
 }
}

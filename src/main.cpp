#include"spider.hpp"

RobotAction spider1;

void setup()
{
  spider1.Start();
}

void loop()
{
  delay(3000);

  spider1.CrawlForward();
  delay(1000);
  spider1.CrawlForward();
  delay(1000);
  spider1.CrawlForward();
  delay(1000);
  spider1.CrawlForward();
  delay(1000);

  while(true);

}

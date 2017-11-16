/// @author Hiroaki Yaguchi JSK

#include <aero_std/spot_list.hh>
#include <gtest/gtest.h>
#include <math.h>

class SpotTest : public::testing::Test {
protected:
  virtual void SetUp() {
    aero::Spot spot;

    spot = aero::MakeSpot(std::string("spot0"),
                          0, 0, 0,
                          0, 0, 0, 1);
    spots_.push_back(spot);
    spot_list_.SaveSpot(spot);

    spot = aero::MakeSpot(std::string("spot1"),
                          1, 0, 0,
                          0, 0, sqrt(0.5), sqrt(0.5));
    spots_.push_back(spot);
    spot_list_.SaveSpot(spot);

    spot = aero::MakeSpot(std::string("spot2"),
                          0, 1, 0,
                          0, 0, sqrt(0.5), -sqrt(0.5));
    spots_.push_back(spot);
    spot_list_.SaveSpot(spot);
  }

  aero::SpotList spot_list_;
  std::vector<aero::Spot> spots_;
};

TEST_F(SpotTest, WriteRead) {
  ASSERT_EQ(spots_.size(), spot_list_.spots().size());

  std::string filename("/tmp/test.yaml");
  // write
  spot_list_.WriteIntoFile(filename);
  // read
  spot_list_.ReadFromFile(filename);

  ASSERT_EQ(spots_.size(), spot_list_.spots().size());

  // check all elements
  for (size_t i = 0; i < spots_.size(); i++) {
    aero::Spot& spot_a = spots_[i];
    aero::Spot& spot_b = spot_list_.spots()[i];
    EXPECT_EQ(spot_a.name, spot_b.name);
    EXPECT_FLOAT_EQ(spot_a.pose.position.x, spot_b.pose.position.x);
    EXPECT_FLOAT_EQ(spot_a.pose.position.y, spot_b.pose.position.y);
    EXPECT_FLOAT_EQ(spot_a.pose.position.z, spot_b.pose.position.z);
    EXPECT_FLOAT_EQ(spot_a.pose.orientation.x, spot_b.pose.orientation.x);
    EXPECT_FLOAT_EQ(spot_a.pose.orientation.y, spot_b.pose.orientation.y);
    EXPECT_FLOAT_EQ(spot_a.pose.orientation.z, spot_b.pose.orientation.z);
    EXPECT_FLOAT_EQ(spot_a.pose.orientation.w, spot_b.pose.orientation.w);
  }
}

TEST_F(SpotTest, SaveDelete) {
  ASSERT_EQ(spots_.size(), spot_list_.spots().size());

  // overwrite
  std::string nname("spot0");
  float nx = 1.0;
  float ny = 1.0;

  aero::Spot spot = aero::MakeSpot(nname,
                                   nx, ny, 0,
                                   0, 0, 0, 1);
  spot_list_.SaveSpot(spot);

  ASSERT_EQ(spots_.size(), spot_list_.spots().size());

  aero::Spot& spot_a = spot_list_.spots()[spot_list_.GetIndex(nname)];
  EXPECT_FLOAT_EQ(spot_a.pose.position.x, nx);
  EXPECT_FLOAT_EQ(spot_a.pose.position.y, ny);

  // add
  std::string aname("spotx");
  aero::Spot aspot = aero::MakeSpot(aname,
                                    0, 0, 0,
                                    0, 0, 0, 1);
  spot_list_.SaveSpot(aspot);
  ASSERT_EQ(spots_.size() + 1, spot_list_.spots().size());

  // delete
  spot_list_.DeleteSpot(aname);
  ASSERT_EQ(spots_.size(), spot_list_.spots().size());
  ASSERT_EQ(spot_list_.GetIndex(aname), -1);
}

/////////////////////////
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

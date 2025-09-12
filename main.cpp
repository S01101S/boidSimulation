#include <iostream>
#include <random>
#include <SFML/Graphics.hpp>
#include <vector>

struct Star
{
    sf::Vector2f position;
    float radius;
};

class Boid
{
public:
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;
    sf::ConvexShape shape;

    Boid(float x, float y, std::mt19937& generator)
    {
        std::uniform_real_distribution<float> distribution(-2.f, 2.f);


        position = sf::Vector2f(x, y);
        velocity = sf::Vector2f(distribution(generator), distribution(generator));
        acceleration = sf::Vector2f(0.f, 0.f);

        const float sizeScale = generateSize(generator);
        sf::Vector2f newSize (15.f * sizeScale, 2.f * sizeScale);
        shape.setPointCount(3);
        shape.setPoint(0,sf::Vector2f(10, 0));
        shape.setPoint(1, sf::Vector2f(-5, -5));
        shape.setPoint(2, sf::Vector2f(-5, 5));
        shape.setOrigin(0, 0);
        shape.setScale(sizeScale, sizeScale);
        shape.setFillColor(sf::Color(generateColor(generator), generateColor(generator), generateColor(generator)));
        shape.setPosition(position);
    }

    int generateColor(std::mt19937& generator)
    {
        std::uniform_int_distribution<int> distribution(0, 255);
        return distribution(generator);
    }

    float generateSize(std::mt19937& generator)
    {
        std::uniform_real_distribution<float> distribution(0.8f, 1.2f);
        return distribution(generator);
    }

    void updateBoidPosition(float windowWidth, float windowHeight)
    {
        velocity += acceleration;

        const float velocityMagnitude = sqrt((velocity.x * velocity.x) + (velocity.y * velocity.y));

        if (velocityMagnitude > 5.f)
        {
            velocity = (velocity / velocityMagnitude) * 5.f;
        }

        position += velocity;
        acceleration *= 0.f;

        float rotationAngle = atan2(velocity.y, velocity.x) * (180.f / M_PI);

        shape.setPosition(position);
        shape.setRotation(rotationAngle);

        if (position.x < 0)
        {
            position.x += windowWidth;
        }
        else if (position.x > windowWidth)
        {
            position.x -= windowWidth;
        }

        if (position.y < 0)
        {
            position.y += windowHeight;
        }
        else if (position.y > windowHeight)
        {
            position.y -= windowHeight;
        }
    }

    void flockRules(std::vector<Boid>& boids, sf::Vector2f mousePosition, bool blackHoleActive, sf::Vector2f blackHolePosition, bool cohesionActive, bool separationActive, bool alignmentActive, std::map<int, std::vector<Boid*>>& spatialGrid, float windowWidth, float cellSize)
    {
        sf::Vector2f totalPosition = sf::Vector2f(0.f, 0.f);
        sf::Vector2f totalVelocity = sf::Vector2f(0.f, 0.f);

        int currentBoidRow = this->position.x / cellSize;
        int currentBoidCol = this->position.y / cellSize;

        std::vector<Boid*> localNeighbours;

        for (int i=currentBoidRow-1; i<=currentBoidRow+1; i++)
        {
            for (int j=currentBoidCol-1; j<=currentBoidCol+1; j++)
            {
                int key = i * (windowWidth / cellSize) + j;

                if (spatialGrid.count(key))
                {
                    std::vector<Boid*>& boidsInCell = spatialGrid.at(key);

                    for (auto* neighbours : boidsInCell)
                    {
                        localNeighbours.push_back(neighbours);
                    }
                }
            }

        }

        const float mouseDx = mousePosition.x - this->position.x;
        const float mouseDy = mousePosition.y - this->position.y;
        float mouseDistance = sqrt((mouseDx * mouseDx) + (mouseDy * mouseDy));

        const float blackHoleDx = blackHolePosition.x - this->position.x;
        const float blackHoleDy = blackHolePosition.y - this->position.y;
        float blackHoleDistance = sqrt((blackHoleDx * blackHoleDx) + (blackHoleDy * blackHoleDy));

        if (blackHoleActive)
        {
            if (blackHoleDistance < 150.f)
            {
                sf::Vector2f attractForce = blackHolePosition - this->position;
                float magnitudeOfAttractForce = sqrt((attractForce.x * attractForce.x) + (attractForce.y * attractForce.y));

                if (magnitudeOfAttractForce > 0)
                {
                    sf::Vector2f normalisedAttractForce = attractForce / magnitudeOfAttractForce;
                    normalisedAttractForce *= 2.f;
                    this->acceleration += normalisedAttractForce;
                }
            }
        }

        if (mouseDistance < 100.f && blackHoleActive == false)
        {
            sf::Vector2f fleeDirection = this->position - mousePosition;
            float magnitudeOfFleeDirection = sqrt((fleeDirection.x * fleeDirection.x) + (fleeDirection.y * fleeDirection.y));

            if (magnitudeOfFleeDirection > 0)
            {
                sf::Vector2f normalisedFleeingdirection = fleeDirection / magnitudeOfFleeDirection;
                normalisedFleeingdirection *= 1.f;
                this->acceleration += normalisedFleeingdirection;
            }
        }
        else
        {
            if (!localNeighbours.empty())
            {
                for (auto& boid : localNeighbours)
                {
                    totalPosition += boid->position;
                }

                if (cohesionActive)
                {
                    sf::Vector2f centerOfFlock = totalPosition / static_cast<float>(localNeighbours.size());
                    sf::Vector2f steeringDirection = centerOfFlock - position;
                    float magnitudeOfSteeringDirection = sqrt(steeringDirection.x*steeringDirection.x + steeringDirection.y*steeringDirection.y);

                    if (magnitudeOfSteeringDirection > 0)
                    {
                        sf::Vector2f normalisedSteeringDirectionForce = steeringDirection / magnitudeOfSteeringDirection;
                        normalisedSteeringDirectionForce *= 0.05f;
                        acceleration += normalisedSteeringDirectionForce;
                    }
                }

                for (auto& neighbourBoid : localNeighbours)
                {
                    if (this == neighbourBoid)
                    {
                        continue;
                    }

                    if (separationActive)
                    {
                        float dx = (this->position.x - neighbourBoid->position.x);
                        float dy = (this->position.y - neighbourBoid->position.y);
                        float distance = sqrt((dx * dx) + (dy * dy));

                        if (distance < 25.f)
                        {
                            sf::Vector2f steerAwayDirection = this->position - neighbourBoid->position;
                            float magnitudeOfSteerAwayDirection = sqrt(steerAwayDirection.x * steerAwayDirection.x + steerAwayDirection.y * steerAwayDirection.y);

                            if (magnitudeOfSteerAwayDirection > 0)
                            {
                                sf::Vector2f normalisedSteerAwayDirection = steerAwayDirection / magnitudeOfSteerAwayDirection;
                                normalisedSteerAwayDirection *= 0.05f;
                                this->acceleration += normalisedSteerAwayDirection;
                            }
                        }
                    }

                }


                for (auto& boid : localNeighbours)
                {
                    totalVelocity += boid->velocity;
                }

                if (alignmentActive)
                {
                    sf::Vector2f averageVelocity = totalVelocity / static_cast<float>(localNeighbours.size());
                    sf::Vector2f velocitySteering = averageVelocity - this->velocity;
                    float magnitudeOfVelocitySteering = sqrt(velocitySteering.x * velocitySteering.x + velocitySteering.y * velocitySteering.y);
                    if (magnitudeOfVelocitySteering > 0)
                    {
                        sf::Vector2f normalisedVelocitySteering = velocitySteering / magnitudeOfVelocitySteering;
                        normalisedVelocitySteering *= 0.05f;
                        this->acceleration += normalisedVelocitySteering;
                    }
                }
            }
        }


        const float accelerationMagnitude = sqrt((this->acceleration.x * this->acceleration.x) + (this->acceleration.y * this->acceleration.y));

        if (accelerationMagnitude > 0.5f)
        {
            this->acceleration = (this->acceleration / accelerationMagnitude) * 0.5f;
        }

    }
};




class UserInterface
{

public:
    sf::Font myFont;
    sf::Text boidCountText;
    sf::Text cohesionActiveText;
    sf::Text separationActiveText;
    sf::Text alignmentActiveText;
    sf::Text addBoidCountText;
    sf::Text minusBoidCountText;
    sf::RectangleShape cohesionButton;
    sf::RectangleShape separationButton;
    sf::RectangleShape alignmentButton;
    sf::RectangleShape minusBoidCountButton;
    sf::RectangleShape addBoidCountButton;


    UserInterface()
    {
        if (!myFont.loadFromFile("arial.ttf"))
        {
            std::cout << "Error loading arial.ttf" << std::endl;
            exit(1);
        }

        setUpText(boidCountText);
        setUpText(cohesionActiveText);
        setUpText(separationActiveText);
        setUpText(alignmentActiveText);
        setUpTextForAddAndMinus(addBoidCountText);
        setUpTextForAddAndMinus(minusBoidCountText);


        separationButton.setSize(sf::Vector2f(15, 15));
        separationButton.setPosition(150, 30);

        cohesionButton.setSize(sf::Vector2f(15, 15));
        cohesionButton.setPosition(150, 50);

        alignmentButton.setSize(sf::Vector2f(15, 15));
        alignmentButton.setPosition(150, 70);

        addBoidCountButton.setSize(sf::Vector2f(10, 10));
        addBoidCountButton.setPosition(120, 13);

        minusBoidCountButton.setSize(sf::Vector2f(10, 10));
        minusBoidCountButton.setPosition(100, 13);
    }

    void update(int boidCount, bool cohesionActive, bool separationActive, bool alignmentActive)
    {
        boidCountText.setString("Boid Count: " + std::to_string(boidCount));
        boidCountText.setPosition(10, 10);

        addBoidCountText.setString("+");
        addBoidCountText.setPosition(120, 9);

        minusBoidCountText.setString("-");
        minusBoidCountText.setPosition(100, 9);

        separationActiveText.setString(std::string("Separation Active: ") + (separationActive ? "ON" : "OFF"));
        separationActiveText.setPosition(10, 30);

        cohesionActiveText.setString(std::string("Cohesion Active: ") + (cohesionActive ? "ON" : "OFF"));
        cohesionActiveText.setPosition(10, 50);

        alignmentActiveText.setString(std::string("Alignment Active: ") + (alignmentActive ? "ON" : "OFF"));
        alignmentActiveText.setPosition(10, 70);

        if (cohesionActive == true)
        {
            cohesionButton.setFillColor(sf::Color::Green);
        }
        else if (cohesionActive == false)
        {
            cohesionButton.setFillColor(sf::Color::Red);
        }

        if (separationActive == true)
        {
            separationButton.setFillColor(sf::Color::Green);
        }
        else if (separationActive == false)
        {
            separationButton.setFillColor(sf::Color::Red);
        }

        if (alignmentActive == true)
        {
            alignmentButton.setFillColor(sf::Color::Green);
        }

        else if (alignmentActive == false)
        {
            alignmentButton.setFillColor(sf::Color::Red);
        }
    }

    int handleClick(sf::Event& event, bool& cohesionActive, bool& separationActive, bool& alignmentActive)
    {

        if (event.type == sf::Event::MouseButtonPressed)
        {
            if (cohesionButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y))
            {
                cohesionActive = !cohesionActive;
                return 2;
            }

            if (separationButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y))
            {
                separationActive = !separationActive;
                return 2;
            }

            if (alignmentButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y))
            {
                alignmentActive = !alignmentActive;
                return 2;
            }

            if (addBoidCountButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y))
            {
                return 1;
            }

            if (minusBoidCountButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y))
            {
                return -1;
            }
        }

        return 0;
    }

    void draw(sf::RenderWindow& window)
    {
        window.draw(boidCountText);
        window.draw(separationActiveText);
        window.draw(cohesionActiveText);
        window.draw(alignmentActiveText);

        window.draw(cohesionButton);
        window.draw(separationButton);
        window.draw(alignmentButton);
        window.draw(addBoidCountButton);
        window.draw(minusBoidCountButton);

        window.draw(addBoidCountText);
        window.draw(minusBoidCountText);
    }


private:
    void setUpText(sf::Text& text)
    {
        text.setFont(myFont);
        text.setCharacterSize(13);
        text.setFillColor(sf::Color::White);
    }

    void setUpTextForAddAndMinus(sf::Text& text)
    {
        text.setFont(myFont);
        text.setCharacterSize(15);
        text.setFillColor(sf::Color::Black);
    }

};




int main()
{
    const float windowHeight = 600.f;
    const float windowWidth = 800.f;
    const float cellSize = 75.f;
    int boidCount = 50;
    bool cohesionActive = true;
    bool separationActive = true;
    bool alignmentActive = true;
    UserInterface boidUI;
    std::map<int, std::vector<Boid*>> spatialGrid;
    sf::RenderTexture renderTexture;
    renderTexture.create(windowWidth, windowHeight);

    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> distributionX(0, windowWidth);
    std::uniform_real_distribution<float> distributionY(0, windowHeight);


    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Boid Simulation");
    window.setFramerateLimit(60);

    std::vector<Boid> boidList;

    for (int i=0; i<boidCount; i++)
    {
        const float boidXPosition = distributionX(generator);
        const float boidYPosition = distributionY(generator);
        Boid newBoid(boidXPosition, boidYPosition, generator);
        boidList.push_back(newBoid);
    }

    std::vector<sf::CircleShape> starField;
    std::uniform_real_distribution<float> starDistributionX(0, windowWidth);
    std::uniform_real_distribution<float> starDistributionY(0, windowHeight);
    std::uniform_real_distribution<float> starRadius(1.f, 1.5f);

    for (int i=0; i<100; i++)
    {
        sf::CircleShape newStar;

        float starXPosition = starDistributionX(generator);
        float starYPosition = starDistributionY(generator);
        float radiusOfStar = starRadius(generator);

        newStar.setPosition(starXPosition, starYPosition);
        newStar.setRadius(radiusOfStar);

        starField.push_back(newStar);
    }


    bool blackHoleActive = false;
    sf::Vector2f blackHolePosition;

    sf::CircleShape blackHoleEffect(30.f);
    blackHoleEffect.setFillColor(sf::Color::Black);
    blackHoleEffect.setOutlineColor(sf::Color(144, 199, 240));
    blackHoleEffect.setOutlineThickness(2.f);
    blackHoleEffect.setOrigin(30.f, 30.f);

    sf::RectangleShape accretionDisk;
    accretionDisk.setSize(sf::Vector2f(100.f, 3.f));
    accretionDisk.setFillColor(sf::Color(42, 101, 145));
    accretionDisk.setOrigin(50.f, 1.f);
    float accretionDiskRotation = 0.f;

    sf::RectangleShape fadeRect(sf::Vector2f(windowWidth, windowHeight));
    fadeRect.setFillColor(sf::Color(0, 0, 0, 50));

    while (window.isOpen())
    {
        sf::Event event;
        spatialGrid.clear();

        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }

            int uiSwitchToggle = boidUI.handleClick(event, cohesionActive, separationActive, alignmentActive);

            if (uiSwitchToggle == 0)
            {
                if (event.type == sf::Event::MouseButtonPressed)
                {
                    if (event.mouseButton.button == sf::Mouse::Left)
                    {
                        blackHoleActive = true;
                        sf::Vector2i mousePixelPosition(event.mouseButton.x, event.mouseButton.y);
                        blackHolePosition = window.mapPixelToCoords(mousePixelPosition);

                    }
                }

                if (event.type == sf::Event::MouseButtonReleased)
                {
                    blackHoleActive = false;
                }
            }

            else if (uiSwitchToggle == 1)
            {
                const float addNewBoidXPosition = distributionX(generator);
                const float addNewBoidYPosition = distributionY(generator);
                Boid newBoid(addNewBoidXPosition, addNewBoidYPosition, generator);
                boidList.push_back(newBoid);

                boidCount = boidList.size();
            }

            else if (uiSwitchToggle == -1)
            {
                boidList.pop_back();
                boidCount = boidList.size();
            }

        }

        sf::Vector2i pixelPosition = sf::Mouse::getPosition(window);
        sf::Vector2f mousePosition = window.mapPixelToCoords(pixelPosition);


        for (auto& currentBoid : boidList)
        {
            int boidRow = currentBoid.position.x / cellSize;
            int boidCol = currentBoid.position.y / cellSize;

            int boidKey = boidRow * (windowWidth / cellSize) + boidCol;
            spatialGrid[boidKey].push_back(&currentBoid);
        }


        for (auto& currentBoid : boidList)
        {
            currentBoid.flockRules(boidList, mousePosition, blackHoleActive, blackHolePosition, cohesionActive, separationActive, alignmentActive, spatialGrid, windowWidth, cellSize);
            currentBoid.updateBoidPosition(windowWidth, windowHeight);
        }

        renderTexture.draw(fadeRect);
        if (blackHoleActive)
        {
            blackHoleEffect.setPosition(blackHolePosition);
            accretionDisk.setPosition(blackHolePosition);
            accretionDiskRotation += 0.5f;
            accretionDisk.setRotation(accretionDiskRotation);
            renderTexture.draw(blackHoleEffect);
            renderTexture.draw(accretionDisk);
        }

        for (auto& star : starField)
        {
            renderTexture.draw(star);
        }

        for (auto& boid : boidList)
        {
            renderTexture.draw(boid.shape);
        }

        renderTexture.display();

        window.clear(sf::Color::Black);

        sf::Sprite trailSprite(renderTexture.getTexture());
        window.draw(trailSprite);

        boidUI.update(boidCount, cohesionActive, separationActive, alignmentActive);
        boidUI.draw(window);

        window.display();


    }

    return 0;

}
